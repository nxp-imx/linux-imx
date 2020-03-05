/** @file  mlanevent.c
 *
 *  @brief Program to receive events from the driver/firmware of the uAP
 *         driver.
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
/****************************************************************************
Change log:
    03/18/08: Initial creation
****************************************************************************/

/****************************************************************************
        Header files
****************************************************************************/
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <ctype.h>
#include <signal.h>
#include <time.h>
#include <sys/time.h>
#include <getopt.h>

#include <sys/socket.h>
#include <linux/netlink.h>
#include <linux/if.h>
#include "mlanevent.h"
#ifdef WIFI_DIRECT_SUPPORT
#include <arpa/inet.h>
#endif

/****************************************************************************
        Definitions
****************************************************************************/
/** Enable or disable debug outputs */
#define DEBUG   1

int debug_level = MSG_NONE;
/****************************************************************************
        Global variables
****************************************************************************/
/** Termination flag */
int terminate_flag = 0;

/****************************************************************************
        Local functions
****************************************************************************/
#if DEBUG
/**
 * @brief           Conditional printf
 *
 * @param level     Severity level of the message
 * @param fmt       Printf format string, followed by optional arguments
 */
void
mlanevent_printf(int level, char *fmt, ...)
{
	va_list ap;
	va_start(ap, fmt);
	if (level <= debug_level) {
		vprintf(fmt, ap);
	}
	va_end(ap);
}

static void
mlanevent_hexdump(int level, void *p, t_s32 len, char delim)
{
	t_s32 i;
	t_u8 *s = p;
	if (level <= debug_level) {
		for (i = 0; i < len; i++) {
			if (i != len - 1)
				printf("%02x%c", *s++, delim);
			else
				printf("%02x\n", *s);
			if ((i + 1) % 16 == 0)
				printf("\n");
		}
	}
}
#endif

/**
 *  @brief Signal handler
 *
 *  @param sig      Received signal number
 *  @return         N/A
 */
void
sig_handler(int sig)
{
	printf("Stopping application.\n");
#if DEBUG
	mlanevent_printf(MSG_DEBUG, "Process ID of process killed = %d\n",
			 getpid());
#endif
	terminate_flag = 1;
}

/**
 *  @brief Dump hex data
 *
 *  @param p        A pointer to data buffer
 *  @param len      The len of data buffer
 *  @param delim    Deliminator character
 *  @return         Hex integer
 */
static void
hexdump(void *p, t_s32 len, char delim)
{
	t_s32 i;
	t_u8 *s = p;
	for (i = 0; i < len; i++) {
		if (i != len - 1)
			printf("%02x%c", *s++, delim);
		else
			printf("%02x\n", *s);
		if ((i + 1) % 16 == 0)
			printf("\n");
	}
}

/**
 *  @brief      Hex to number
 *
 *  @param c    Hex value
 *  @return     Integer value or -1
 */
int
hex2num(char c)
{
	if (c >= '0' && c <= '9')
		return c - '0';
	if (c >= 'a' && c <= 'f')
		return c - 'a' + 10;
	if (c >= 'A' && c <= 'F')
		return c - 'A' + 10;

	return -1;
}

/**
 *  @brief      Check hex string
 *
 *  @param hex      A pointer to hex string
 *  @return         0 or MLAN_EVENT_FAILURE
 */
int
ishexstring(void *hex)
{
	int i, a;
	char *p = hex;
	int len = strlen(p);
	if (!strncasecmp("0x", p, 2)) {
		p += 2;
		len -= 2;
	}
	for (i = 0; i < len; i++) {
		a = hex2num(*p);
		if (a < 0)
			return MLAN_EVENT_FAILURE;
		p++;
	}
	return 0;
}

/**
 *    @brief Convert char to hex integer
 *
 *    @param chr          Char
 *    @return             Hex integer
 */
unsigned char
hexc2bin(char chr)
{
	if (chr >= '0' && chr <= '9')
		chr -= '0';
	else if (chr >= 'A' && chr <= 'F')
		chr -= ('A' - 10);
	else if (chr >= 'a' && chr <= 'f')
		chr -= ('a' - 10);

	return chr;
}

/**
 *    @brief Convert string to hex integer
 *
 *    @param s            A pointer string buffer
 *    @return             Hex integer
 */
unsigned int
a2hex(char *s)
{
	unsigned int val = 0;
	if (!strncasecmp("0x", s, 2)) {
		s += 2;
	}
	while (*s && isxdigit(*s)) {
		val = (val << 4) + hexc2bin(*s++);
	}
	return val;
}

/**
 *  @brief Prints a MAC address in colon separated form from raw data
 *
 *  @param raw      A pointer to the hex data buffer
 *  @return         N/A
 */
void
print_mac(t_u8 *raw)
{
	printf("%02x:%02x:%02x:%02x:%02x:%02x", (unsigned int)raw[0],
	       (unsigned int)raw[1], (unsigned int)raw[2], (unsigned int)raw[3],
	       (unsigned int)raw[4], (unsigned int)raw[5]);
	return;
}

/**
 *  @brief Print usage information
 *
 *  @return         N/A
 */
void
print_usage(void)
{
	printf("\n");
	printf("Usage : mlanevent.exe [-v] [-h]\n");
	printf("    -v               : Print version information\n");
	printf("    -h               : Print help information\n");
	printf("    -d               : Set debug level: 0|1|2\n");
	printf("    -i               : Specify device number from 0 to %d\n",
	       MAX_NO_OF_DEVICES - 1);
	printf("                       0xff for all devices\n");
	printf("\n");
}

/**
 *  @brief Parse and print STA deauthentication event data
 *
 *  @param buffer   Pointer to received event buffer
 *  @param size     Length of the received event data
 *  @return         N/A
 */
void
print_event_sta_deauth(t_u8 *buffer, t_u16 size)
{
	eventbuf_sta_deauth *event_body = NULL;

	if (size < sizeof(eventbuf_sta_deauth)) {
		printf("ERR:Event buffer too small!\n");
		return;
	}
	event_body = (eventbuf_sta_deauth *)buffer;
	event_body->reason_code = uap_le16_to_cpu(event_body->reason_code);
	printf("EVENT: STA_DEAUTH\n");
	printf("Deauthenticated STA MAC: ");
	print_mac(event_body->sta_mac_address);
	printf("\nReason: ");
	switch (event_body->reason_code) {
	case 1:
		printf("Client station leaving the network\n");
		break;
	case 2:
		printf("Client station aged out\n");
		break;
	case 3:
		printf("Client station deauthenticated by user's request\n");
		break;
	case 4:
		printf("Client station authentication failure\n");
		break;
	case 5:
		printf("Client station association failure\n");
		break;
	case 6:
		printf("Client mac address is blocked by ACL filter\n");
		break;
	case 7:
		printf("Client station table is full\n");
		break;
	case 8:
		printf("Client 4-way handshake timeout\n");
		break;
	case 9:
		printf("Client group key handshake timeout\n");
		break;
	default:
		printf("Unspecified\n");
		break;
	}
	return;
}

/**
 *  @brief Parse and print WEP ICV error event data
 *
 *  @param buffer   Pointer to received event buffer
 *  @param size     Length of the received event data
 *  @return         N/A
 */
void
print_event_wep_icv_error(t_u8 *buffer, t_u16 size)
{
	int i = 0;
	eventbuf_wep_icv_error *event_body = NULL;

	if (size < sizeof(eventbuf_wep_icv_error)) {
		printf("ERR:Event buffer too small!\n");
		return;
	}
	event_body = (eventbuf_wep_icv_error *)buffer;
	event_body->reason_code = uap_le16_to_cpu(event_body->reason_code);
	printf("EVENT: WEP_ICV_ERROR\n");
	printf("Deauthenticated STA MAC: ");
	print_mac(event_body->sta_mac_address);
	printf("WEP key index = %d\n", event_body->wep_key_index);
	printf("WEP key length = %d\n", event_body->wep_key_length);
	printf("WEP key : \n");
	for (i = 0; i < event_body->wep_key_length; i++) {
		printf("%02x ", event_body->wep_key[i]);
	}
	printf("\n");
	return;
}

/**
 *  @brief Prints mgmt frame
 *
 *  @param mgmt_tlv A pointer to mgmt_tlv
 *  @param tlv_len  Length of tlv payload
 *  @return         N/A
 */
void
print_mgmt_frame(MrvlIETypes_MgmtFrameSet_t *mgmt_tlv, int tlv_len)
{
	IEEEtypes_AssocRqst_t *assoc_req = NULL;
	IEEEtypes_ReAssocRqst_t *reassoc_req = NULL;
	IEEEtypes_AssocRsp_t *assoc_resp = NULL;
	t_u16 frm_ctl = 0;
	printf("\nMgmt Frame:\n");
	memcpy(&frm_ctl, &mgmt_tlv->frame_control, sizeof(t_u16));
	printf("FrameControl: 0x%x\n", frm_ctl);
	if (mgmt_tlv->frame_control.type != 0) {
		printf("Frame type=%d subtype=%d:\n",
		       mgmt_tlv->frame_control.type,
		       mgmt_tlv->frame_control.sub_type);
		hexdump(mgmt_tlv->frame_contents, tlv_len - sizeof(t_u16), ' ');
		return;
	}
	switch (mgmt_tlv->frame_control.sub_type) {
	case SUBTYPE_ASSOC_REQUEST:
		printf("Assoc Request:\n");
		assoc_req = (IEEEtypes_AssocRqst_t *)mgmt_tlv->frame_contents;
		printf("CapInfo: 0x%x  ListenInterval: 0x%x \n",
		       uap_le16_to_cpu(assoc_req->cap_info),
		       uap_le16_to_cpu(assoc_req->listen_interval));
		printf("AssocReqIE:\n");
		hexdump(assoc_req->ie_buffer,
			tlv_len - sizeof(IEEEtypes_AssocRqst_t)
			- sizeof(IEEEtypes_FrameCtl_t), ' ');
		break;
	case SUBTYPE_REASSOC_REQUEST:
		printf("ReAssoc Request:\n");
		reassoc_req =
			(IEEEtypes_ReAssocRqst_t *)mgmt_tlv->frame_contents;
		printf("CapInfo: 0x%x  ListenInterval: 0x%x \n",
		       uap_le16_to_cpu(reassoc_req->cap_info),
		       uap_le16_to_cpu(reassoc_req->listen_interval));
		printf("Current AP address: ");
		print_mac(reassoc_req->current_ap_addr);
		printf("\nReAssocReqIE:\n");
		hexdump(reassoc_req->ie_buffer,
			tlv_len - sizeof(IEEEtypes_ReAssocRqst_t)
			- sizeof(IEEEtypes_FrameCtl_t), ' ');
		break;
	case SUBTYPE_ASSOC_RESPONSE:
	case SUBTYPE_REASSOC_RESPONSE:
		if (mgmt_tlv->frame_control.sub_type == SUBTYPE_ASSOC_RESPONSE)
			printf("Assoc Response:\n");
		else
			printf("ReAssoc Response:\n");
		assoc_resp = (IEEEtypes_AssocRsp_t *)mgmt_tlv->frame_contents;
		printf("CapInfo: 0x%x  StatusCode: %d  AID: 0x%x \n",
		       uap_le16_to_cpu(assoc_resp->cap_info),
		       (int)(uap_le16_to_cpu(assoc_resp->status_code)),
		       uap_le16_to_cpu(assoc_resp->aid) & 0x3fff);
		break;
	default:
		printf("Frame subtype = %d:\n",
		       mgmt_tlv->frame_control.sub_type);
		hexdump(mgmt_tlv->frame_contents, tlv_len - sizeof(t_u16), ' ');
		break;
	}
	return;
}

/**
 *  @brief Parse and print RSN connect event data
 *
 *  @param buffer   Pointer to received buffer
 *  @param size     Length of the received event data
 *  @return         N/A
 */
void
print_event_rsn_connect(t_u8 *buffer, t_u16 size)
{
	int tlv_buf_left = size;
	t_u16 tlv_type, tlv_len;
	tlvbuf_header *tlv = NULL;
	eventbuf_rsn_connect *event_body = NULL;
	if (size < sizeof(eventbuf_rsn_connect)) {
		printf("ERR:Event buffer too small!\n");
		return;
	}
	event_body = (eventbuf_rsn_connect *)buffer;
	printf("EVENT: RSN_CONNECT\n");
	printf("Station MAC: ");
	print_mac(event_body->sta_mac_address);
	printf("\n");
	tlv_buf_left = size - sizeof(eventbuf_rsn_connect);
	if (tlv_buf_left < (int)sizeof(tlvbuf_header))
		return;
	tlv = (tlvbuf_header *)(buffer + sizeof(eventbuf_rsn_connect));

	while (tlv_buf_left >= (int)sizeof(tlvbuf_header)) {
		tlv_type = uap_le16_to_cpu(tlv->type);
		tlv_len = uap_le16_to_cpu(tlv->len);
		if ((sizeof(tlvbuf_header) + tlv_len) >
		    (unsigned int)tlv_buf_left) {
			printf("wrong tlv: tlvLen=%d, tlvBufLeft=%d\n", tlv_len,
			       tlv_buf_left);
			break;
		}
		switch (tlv_type) {
		case IEEE_WPA_IE:
			printf("WPA IE:\n");
			hexdump((t_u8 *)tlv + sizeof(tlvbuf_header), tlv_len,
				' ');
			break;
		case IEEE_RSN_IE:
			printf("RSN IE:\n");
			hexdump((t_u8 *)tlv + sizeof(tlvbuf_header), tlv_len,
				' ');
			break;
		default:
			printf("unknown tlv: %d\n", tlv_type);
			break;
		}
		tlv_buf_left -= (sizeof(tlvbuf_header) + tlv_len);
		tlv = (tlvbuf_header *)((t_u8 *)tlv + tlv_len +
					sizeof(tlvbuf_header));
	}
	return;
}

/**
 *  @brief Parse and print STA associate event data
 *
 *  @param buffer   Pointer to received buffer
 *  @param size     Length of the received event data
 *  @return         N/A
 */
void
print_event_sta_assoc(t_u8 *buffer, t_u16 size)
{
	int tlv_buf_left = size;
	t_u16 tlv_type, tlv_len;
	tlvbuf_header *tlv = NULL;
	MrvlIEtypes_WapiInfoSet_t *wapi_tlv = NULL;
	MrvlIETypes_MgmtFrameSet_t *mgmt_tlv = NULL;
	eventbuf_sta_assoc *event_body = NULL;
	if (size < sizeof(eventbuf_sta_assoc)) {
		printf("ERR:Event buffer too small!\n");
		return;
	}
	event_body = (eventbuf_sta_assoc *)buffer;
	printf("EVENT: STA_ASSOCIATE\n");
	printf("Associated STA MAC: ");
	print_mac(event_body->sta_mac_address);
	printf("\n");
	tlv_buf_left = size - sizeof(eventbuf_sta_assoc);
	if (tlv_buf_left < (int)sizeof(tlvbuf_header))
		return;
	tlv = (tlvbuf_header *)(buffer + sizeof(eventbuf_sta_assoc));

	while (tlv_buf_left >= (int)sizeof(tlvbuf_header)) {
		tlv_type = uap_le16_to_cpu(tlv->type);
		tlv_len = uap_le16_to_cpu(tlv->len);
		if ((sizeof(tlvbuf_header) + tlv_len) >
		    (unsigned int)tlv_buf_left) {
			printf("wrong tlv: tlvLen=%d, tlvBufLeft=%d\n", tlv_len,
			       tlv_buf_left);
			break;
		}
		switch (tlv_type) {
		case MRVL_WAPI_INFO_TLV_ID:
			wapi_tlv = (MrvlIEtypes_WapiInfoSet_t *)tlv;
			printf("WAPI Multicast PN:\n");
			hexdump(wapi_tlv->multicast_PN, tlv_len, ' ');
			break;
		case MRVL_MGMT_FRAME_TLV_ID:
			mgmt_tlv = (MrvlIETypes_MgmtFrameSet_t *)tlv;
			print_mgmt_frame(mgmt_tlv, tlv_len);
			break;
		default:
			printf("unknown tlv: %d\n", tlv_type);
			break;
		}
		tlv_buf_left -= (sizeof(tlvbuf_header) + tlv_len);
		tlv = (tlvbuf_header *)((t_u8 *)tlv + tlv_len +
					sizeof(tlvbuf_header));
	}
	return;
}

/**
 *  @brief Parse and print BSS start event data
 *
 *  @param buffer   Pointer to received buffer
 *  @param size     Length of the received event data
 *  @return         N/A
 */
void
print_event_bss_start(t_u8 *buffer, t_u16 size)
{
	eventbuf_bss_start *event_body = NULL;
	int tlv_buf_left = size;
	t_u16 tlv_type, tlv_len;
	tlvbuf_header *tlv = NULL;
	tlvbuf_channel_config *channel_tlv = NULL;

	if (size < sizeof(eventbuf_bss_start)) {
		printf("ERR:Event buffer too small!\n");
		return;
	}
	event_body = (eventbuf_bss_start *)buffer;
	printf("EVENT: BSS_START ");
	printf("BSS MAC: ");
	print_mac(event_body->ap_mac_address);
	printf("\n");
	tlv_buf_left = size - sizeof(eventbuf_bss_start);
	if (tlv_buf_left < (int)sizeof(tlvbuf_header))
		return;
	tlv = (tlvbuf_header *)(buffer + sizeof(eventbuf_bss_start));

	while (tlv_buf_left >= (int)sizeof(tlvbuf_header)) {
		tlv_type = uap_le16_to_cpu(tlv->type);
		tlv_len = uap_le16_to_cpu(tlv->len);
		if ((sizeof(tlvbuf_header) + tlv_len) >
		    (unsigned int)tlv_buf_left) {
			printf("wrong tlv: tlvLen=%d, tlvBufLeft=%d\n", tlv_len,
			       tlv_buf_left);
			break;
		}
		switch (tlv_type) {
		case MRVL_CHANNELCONFIG_TLV_ID:
			channel_tlv = (tlvbuf_channel_config *)tlv;
			printf("Channel = %d\n", channel_tlv->chan_number);
			printf("Band = %s\n",
			       (channel_tlv->bandcfg.chanBand ==
				BAND_5GHZ) ? "5GHz" : "2.4GHz");
			printf("Channel Select Mode = %s\n",
			       (channel_tlv->bandcfg.scanMode ==
				SCAN_MODE_ACS) ? "ACS" : "Manual");
			if (channel_tlv->bandcfg.chan2Offset == SEC_CHAN_NONE)
				printf("no secondary channel\n");
			else if (channel_tlv->bandcfg.chan2Offset ==
				 SEC_CHAN_ABOVE)
				printf("secondary channel is above primary channel\n");
			else if (channel_tlv->bandcfg.chan2Offset ==
				 SEC_CHAN_BELOW)
				printf("secondary channel is below primary channel\n");
			break;
		default:
#if DEBUG
			mlanevent_printf(MSG_DEBUG, "unknown tlv: %d\n",
					 tlv_type);
#endif
			break;
		}
		tlv_buf_left -= (sizeof(tlvbuf_header) + tlv_len);
		tlv = (tlvbuf_header *)((t_u8 *)tlv + tlv_len +
					sizeof(tlvbuf_header));
	}

	return;
}

#ifdef WIFI_DIRECT_SUPPORT
/**
 *  @brief Print WIFI_WPS IE elements from event payload
 *
 *  @param buffer   Pointer to received buffer
 *  @param size     Length of the received event data
 *  @return         N/A
 */
void
print_wifi_wps_ie_elements(t_u8 *buffer, t_u16 size)
{
	t_u8 *ptr = buffer;
	t_u8 *array_ptr;
	int i;
	t_u16 wps_len = 0, wps_type = 0;
	t_u16 ie_len_wps = size;

	while (ie_len_wps > sizeof(tlvbuf_wps_ie)) {
		memcpy(&wps_type, ptr, sizeof(t_u16));
		memcpy(&wps_len, ptr + 2, sizeof(t_u16));
		endian_convert_tlv_wps_header_in(wps_type, wps_len);
		switch (wps_type) {
		case SC_Version:
			{
				tlvbuf_wps_ie *wps_tlv = (tlvbuf_wps_ie *)ptr;
				printf("\t WPS Version = 0x%2x\n",
				       *(wps_tlv->data));
			}
			break;
		case SC_Simple_Config_State:
			{
				tlvbuf_wps_ie *wps_tlv = (tlvbuf_wps_ie *)ptr;
				printf("\t WPS setupstate = 0x%x\n",
				       *(wps_tlv->data));
			}
			break;
		case SC_Request_Type:
			{
				tlvbuf_wps_ie *wps_tlv = (tlvbuf_wps_ie *)ptr;
				printf("\t WPS RequestType = 0x%x\n",
				       *(wps_tlv->data));
			}
			break;
		case SC_Response_Type:
			{
				tlvbuf_wps_ie *wps_tlv = (tlvbuf_wps_ie *)ptr;
				printf("\t WPS ResponseType = 0x%x\n",
				       *(wps_tlv->data));
			}
			break;
		case SC_Config_Methods:
			{
				t_u16 wps_config_methods = 0;
				tlvbuf_wps_ie *wps_tlv = (tlvbuf_wps_ie *)ptr;
				memcpy(&wps_config_methods, wps_tlv->data,
				       sizeof(t_u16));
				wps_config_methods = ntohs(wps_config_methods);
				printf("\t WPS SpecConfigMethods = 0x%x\n",
				       wps_config_methods);
			}
			break;
		case SC_UUID_E:
			{
				tlvbuf_wps_ie *wps_tlv = (tlvbuf_wps_ie *)ptr;
				array_ptr = wps_tlv->data;
				printf("\t WPS UUID = ");
				for (i = 0; i < wps_len; i++)
					printf("0x%02X ", *array_ptr++);
				printf("\n");
			}
			break;
		case SC_Primary_Device_Type:
			{
				tlvbuf_wps_ie *wps_tlv = (tlvbuf_wps_ie *)ptr;
				array_ptr = wps_tlv->data;
				printf("\t WPS Primary Device Type = ");
				for (i = 0; i < wps_len; i++)
					printf("0x%02X ", *array_ptr++);
				printf("\n");
			}
			break;
		case SC_RF_Band:
			{
				tlvbuf_wps_ie *wps_tlv = (tlvbuf_wps_ie *)ptr;
				printf("\t WPS RF Band = 0x%x\n",
				       *(wps_tlv->data));
			}
			break;
		case SC_Association_State:
			{
				t_u16 wps_association_state = 0;
				tlvbuf_wps_ie *wps_tlv = (tlvbuf_wps_ie *)ptr;
				memcpy(&wps_association_state, wps_tlv->data,
				       sizeof(t_u16));
				wps_association_state =
					ntohs(wps_association_state);
				printf("\t WPS Association State = 0x%x\n",
				       wps_association_state);
			}
			break;
		case SC_Configuration_Error:
			{
				t_u16 wps_configuration_error = 0;
				tlvbuf_wps_ie *wps_tlv = (tlvbuf_wps_ie *)ptr;
				memcpy(&wps_configuration_error, wps_tlv->data,
				       sizeof(t_u16));
				wps_configuration_error =
					ntohs(wps_configuration_error);
				printf("\t WPS Configuration Error = 0x%x\n",
				       wps_configuration_error);
			}
			break;
		case SC_Device_Password_ID:
			{
				t_u16 wps_device_password_id = 0;
				tlvbuf_wps_ie *wps_tlv = (tlvbuf_wps_ie *)ptr;
				memcpy(&wps_device_password_id, wps_tlv->data,
				       sizeof(t_u16));
				wps_device_password_id =
					ntohs(wps_device_password_id);
				printf("\t WPS Device Password ID = 0x%x\n",
				       wps_device_password_id);
			}
			break;
		case SC_Device_Name:
			{
				tlvbuf_wps_ie *wps_tlv = (tlvbuf_wps_ie *)ptr;
				array_ptr = wps_tlv->data;
				printf("\t WPS Device Name = ");
				for (i = 0; i < wps_len; i++)
					printf("%c", *array_ptr++);
				printf("\n");
			}
			break;
		case SC_Manufacturer:
			{
				tlvbuf_wps_ie *wps_tlv = (tlvbuf_wps_ie *)ptr;
				array_ptr = wps_tlv->data;
				printf("\t WPS Manufacturer = ");
				for (i = 0; i < wps_len; i++)
					printf("%c", *array_ptr++);
				printf("\n");
			}
			break;
		case SC_Model_Name:
			{
				tlvbuf_wps_ie *wps_tlv = (tlvbuf_wps_ie *)ptr;
				array_ptr = wps_tlv->data;
				printf("\t WPS Model Name = ");
				for (i = 0; i < wps_len; i++)
					printf("%c", *array_ptr++);
				printf("\n");
			}
			break;
		case SC_Model_Number:
			{
				tlvbuf_wps_ie *wps_tlv = (tlvbuf_wps_ie *)ptr;
				array_ptr = wps_tlv->data;
				printf("\t WPS Model Number = ");
				for (i = 0; i < wps_len; i++)
					printf("%c", *array_ptr++);
				printf("\n");
			}
			break;
		case SC_Serial_Number:
			{
				tlvbuf_wps_ie *wps_tlv = (tlvbuf_wps_ie *)ptr;
				array_ptr = wps_tlv->data;
				printf("\t WPS Serial Number = ");
				for (i = 0; i < wps_len; i++)
					printf("%c", *array_ptr++);
				printf("\n");
			}
			break;
		case SC_Vendor_Extension:
			{
				tlvbuf_wps_ie *wps_tlv = (tlvbuf_wps_ie *)ptr;
				array_ptr = wps_tlv->data;
				printf("\t WPS 2.0 Vendor Extension = ");
				for (i = 0; i < wps_len; i++)
					printf("%x", *array_ptr++);
				printf("\n");
			}
			break;
		case SC_Selected_Registrar:
			{
				tlvbuf_wps_ie *wps_tlv = (tlvbuf_wps_ie *)ptr;
				printf("\t Selected Registrar = %d\n",
				       *(wps_tlv->data));
			}
			break;
		case SC_SelectedRegistrarConfigMethods:
			{
				t_u16 sr_config_methods = 0;
				tlvbuf_wps_ie *wps_tlv = (tlvbuf_wps_ie *)ptr;
				memcpy(&sr_config_methods, wps_tlv->data,
				       sizeof(t_u16));
				sr_config_methods = ntohs(sr_config_methods);
				printf("\t Selected Registrar Configuration Methods = 0x%x\n", sr_config_methods);
			}
			break;
		default:
			printf("unknown ie=0x%x, len=%d\n", wps_type, wps_len);
			break;
		}
		ptr += wps_len + sizeof(tlvbuf_wps_ie);
		/* Take care of error condition */
		if (wps_len + sizeof(tlvbuf_wps_ie) <= ie_len_wps)
			ie_len_wps -= wps_len + sizeof(tlvbuf_wps_ie);
		else
			ie_len_wps = 0;
	}
}

/**
 *  @brief Print WIFIDIRECT IE elements from event payload
 *
 *  @param buffer   Pointer to received buffer
 *  @param size     Length of the received event data
 *  @return         N/A
 */
void
print_wifidirect_ie_elements(t_u8 *buffer, t_u16 size)
{
	t_u8 *ptr = buffer;
	t_u8 *array_ptr, *orig_ptr = NULL;
	int i;
	static t_u16 len = 0;
	static t_u16 saved_len = 0;
	static t_u16 pending_len = 0;
	static t_u8 type = 0;
	static t_u8 next_byte = WIFIDIRECT_OVERLAP_TYPE;
	static t_u8 saved_data[WIFI_IE_MAX_PAYLOAD] = { 0 };
	t_u16 temp;
	t_u16 left_len = size;

	while (left_len > 0) {
		if (next_byte == WIFIDIRECT_OVERLAP_TYPE) {
			type = *ptr;
			next_byte = WIFIDIRECT_OVERLAP_LEN;
			left_len--;
			ptr++;
		}
		if (left_len >= sizeof(len) &&
		    next_byte == WIFIDIRECT_OVERLAP_LEN) {
			memcpy(&len, ptr, sizeof(t_u16));
			len = uap_le16_to_cpu(len);
			next_byte = WIFIDIRECT_OVERLAP_DATA;
			left_len -= sizeof(t_u16);
			ptr += sizeof(t_u16);

			/* case when Type, Len in one frame and data in next */
			if (left_len == 0) {
				memcpy(saved_data,
				       ptr - WIFIDIRECT_IE_HEADER_LEN,
				       WIFIDIRECT_IE_HEADER_LEN);
				saved_len = WIFIDIRECT_IE_HEADER_LEN;
				pending_len = len;
			}
		}
		if (left_len > 0 && next_byte == WIFIDIRECT_OVERLAP_DATA) {
			/* copy next data */
			if (pending_len > 0 &&
			    (left_len <= (WIFI_IE_MAX_PAYLOAD - saved_len))) {
				memcpy(saved_data + saved_len, ptr,
				       pending_len);
				orig_ptr = ptr;
				ptr = saved_data;
			} else {
				ptr -= WIFIDIRECT_IE_HEADER_LEN;
			}

			if (!pending_len && !orig_ptr && left_len < len) {
				/* save along with type and len */
				memcpy(saved_data,
				       ptr - WIFIDIRECT_IE_HEADER_LEN,
				       left_len + WIFIDIRECT_IE_HEADER_LEN);
				saved_len = left_len + WIFIDIRECT_IE_HEADER_LEN;
				pending_len = len - left_len;
				break;
			}
			switch (type) {
			case TLV_TYPE_WIFIDIRECT_DEVICE_ID:
				{
					tlvbuf_wifidirect_device_id
						*wifidirect_tlv =
						(tlvbuf_wifidirect_device_id *)
						ptr;
					printf("\t Device ID - ");
					print_mac(wifidirect_tlv->
						  dev_mac_address);
					printf("\n");
				}
				break;
			case TLV_TYPE_WIFIDIRECT_CAPABILITY:
				{
					tlvbuf_wifidirect_capability
						*wifidirect_tlv =
						(tlvbuf_wifidirect_capability *)
						ptr;
					printf("\t Device capability = %d\n",
					       (int)wifidirect_tlv->
					       dev_capability);
					printf("\t Group capability = %d\n",
					       (int)wifidirect_tlv->
					       group_capability);
				}
				break;
			case TLV_TYPE_WIFIDIRECT_GROUPOWNER_INTENT:
				{
					tlvbuf_wifidirect_group_owner_intent
						*wifidirect_tlv =
						(tlvbuf_wifidirect_group_owner_intent
						 *)ptr;
					printf("\t Group owner intent = %d\n",
					       (int)wifidirect_tlv->dev_intent);
				}
				break;
			case TLV_TYPE_WIFIDIRECT_MANAGEABILITY:
				{
					tlvbuf_wifidirect_manageability
						*wifidirect_tlv =
						(tlvbuf_wifidirect_manageability
						 *)ptr;
					printf("\t Manageability = %d\n",
					       (int)wifidirect_tlv->
					       manageability);
				}
				break;
			case TLV_TYPE_WIFIDIRECT_INVITATION_FLAG:
				{
					tlvbuf_wifidirect_invitation_flag
						*wifidirect_tlv =
						(tlvbuf_wifidirect_invitation_flag
						 *)ptr;
					printf("\t Invitation Flag = %d\n",
					       (int)wifidirect_tlv->
					       invitation_flag &
					       INVITATION_FLAG_MASK);
				}
				break;
			case TLV_TYPE_WIFIDIRECT_CHANNEL_LIST:
				{
					tlvbuf_wifidirect_channel_list
						*wifidirect_tlv =
						(tlvbuf_wifidirect_channel_list
						 *)ptr;
					chan_entry *temp_ptr;
					printf("\t Country String %c%c",
					       wifidirect_tlv->
					       country_string[0],
					       wifidirect_tlv->
					       country_string[1]);
					if (isalpha
					    (wifidirect_tlv->country_string[2]))
						printf("%c",
						       wifidirect_tlv->
						       country_string[2]);
					printf("\n");
					temp_ptr =
						(chan_entry *)wifidirect_tlv->
						wifidirect_chan_entry_list;
					temp = uap_le16_to_cpu(wifidirect_tlv->
							       length) -
						(sizeof
						 (tlvbuf_wifidirect_channel_list)
						 - WIFIDIRECT_IE_HEADER_LEN);
					while (temp) {
						printf("\t Regulatory_class = %d\n", (int)(temp_ptr->regulatory_class));
						printf("\t No of channels = %d\n", (int)temp_ptr->num_of_channels);
						printf("\t Channel list = ");
						for (i = 0;
						     i <
						     temp_ptr->num_of_channels;
						     i++) {
							printf("%d ",
							       *(temp_ptr->
								 chan_list +
								 i));
						}
						printf("\n");
						temp -= sizeof(chan_entry) +
							temp_ptr->
							num_of_channels;
						temp_ptr =
							(chan_entry *)((t_u8 *)
								       temp_ptr
								       +
								       sizeof
								       (chan_entry)
								       +
								       temp_ptr->
								       num_of_channels);
					}
					printf("\n");
				}
				break;
			case TLV_TYPE_WIFIDIRECT_NOTICE_OF_ABSENCE:
				{
					tlvbuf_wifidirect_notice_of_absence
						*wifidirect_tlv =
						(tlvbuf_wifidirect_notice_of_absence
						 *)ptr;
					noa_descriptor *temp_ptr;
					printf("\t Instance of Notice of absence timing %d\n", (int)wifidirect_tlv->noa_index);
					printf("\t CTWindow and Opportunistic power save parameters %d\n", (int)wifidirect_tlv->ctwindow_opp_ps);
					temp_ptr =
						(noa_descriptor *)
						wifidirect_tlv->
						wifidirect_noa_descriptor_list;
					temp = uap_le16_to_cpu(wifidirect_tlv->
							       length) -
						(sizeof
						 (tlvbuf_wifidirect_notice_of_absence)
						 - WIFIDIRECT_IE_HEADER_LEN);
					while (temp) {
						printf("\t Count or Type = %d\n", (int)temp_ptr->count_type);
						printf("\t Duration = %dms\n",
						       uap_le32_to_cpu
						       (temp_ptr->duration));
						printf("\t Interval = %dms\n",
						       uap_le32_to_cpu
						       (temp_ptr->interval));
						printf("\t Start Time = %d\n",
						       uap_le32_to_cpu
						       (temp_ptr->start_time));
						printf("\n");
						temp_ptr +=
							sizeof(noa_descriptor);
						temp -= sizeof(noa_descriptor);
					}
				}
				break;
			case TLV_TYPE_WIFIDIRECT_DEVICE_INFO:
				{
					tlvbuf_wifidirect_device_info
						*wifidirect_tlv =
						(tlvbuf_wifidirect_device_info
						 *)ptr;
					printf("\t Device address - ");
					print_mac(wifidirect_tlv->dev_address);
					printf("\n");
					printf("\t Config methods - 0x%02X\n",
					       ntohs(wifidirect_tlv->
						     config_methods));
					printf("\t Primay device type = %02d-%02X%02X%02X%02X-%02d\n", (int)ntohs(wifidirect_tlv->primary_category), (int)wifidirect_tlv->primary_oui[0], (int)wifidirect_tlv->primary_oui[1], (int)wifidirect_tlv->primary_oui[2], (int)wifidirect_tlv->primary_oui[3], (int)ntohs(wifidirect_tlv->primary_subcategory));
					printf("\t Secondary Device Count = %d\n", (int)wifidirect_tlv->secondary_dev_count);
					array_ptr =
						wifidirect_tlv->
						secondary_dev_info;
					for (i = 0;
					     i <
					     wifidirect_tlv->
					     secondary_dev_count; i++) {
						memcpy(&temp, array_ptr,
						       sizeof(t_u16));
						printf("\t Secondary device type = %02d-", ntohs(temp));
						array_ptr += sizeof(temp);
						printf("%02X%02X%02X%02X",
						       array_ptr[0],
						       array_ptr[1],
						       array_ptr[2],
						       array_ptr[3]);
						array_ptr += 4;
						memcpy(&temp, array_ptr,
						       sizeof(t_u16));
						printf("-%02d\n", ntohs(temp));
						array_ptr += sizeof(temp);
					}
					/* display device name */
					array_ptr =
						wifidirect_tlv->device_name +
						wifidirect_tlv->
						secondary_dev_count *
						WPS_DEVICE_TYPE_LEN;
					if (*(t_u16 *)
					    (((t_u8 *)(&wifidirect_tlv->
						       device_name_len)) +
					     wifidirect_tlv->
					     secondary_dev_count *
					     WPS_DEVICE_TYPE_LEN))
						printf("\t Device Name =  ");
					memcpy(&temp,
					       (((t_u8 *)(&wifidirect_tlv->
							  device_name_len)) +
						wifidirect_tlv->
						secondary_dev_count *
						WPS_DEVICE_TYPE_LEN),
					       sizeof(t_u16));
					temp = ntohs(temp);
					for (i = 0; i < temp; i++)
						printf("%c", *array_ptr++);
					printf("\n");
				}
				break;
			case TLV_TYPE_WIFIDIRECT_GROUP_INFO:
				{
					tlvbuf_wifidirect_group_info
						*wifidirect_tlv =
						(tlvbuf_wifidirect_group_info *)
						ptr;
					t_u8 wifidirect_client_dev_length;
					wifidirect_client_dev_info *temp_ptr;
					temp_ptr =
						(wifidirect_client_dev_info *)
						wifidirect_tlv->
						wifidirect_client_dev_list;
					if (temp_ptr == NULL)
						break;
					wifidirect_client_dev_length =
						temp_ptr->dev_length;
					temp = uap_le16_to_cpu(wifidirect_tlv->
							       length) -
						wifidirect_client_dev_length;
					while (temp_ptr) {

						printf("\t Group WifiDirect Client Device address - ");
						print_mac(temp_ptr->
							  wifidirect_dev_address);
						printf("\n");
						printf("\t Group WifiDirect Client Interface address - ");
						print_mac(temp_ptr->
							  wifidirect_intf_address);
						printf("\n");
						printf("\t Group WifiDirect Client Device capability = %d\n", (int)temp_ptr->wifidirect_dev_capability);
						printf("\t Group WifiDirect Client Config methods - 0x%02X\n", ntohs(temp_ptr->config_methods));
						printf("\t Group WifiDirect Client Primay device type = %02d-%02X%02X%02X%02X-%02d\n", (int)ntohs(temp_ptr->primary_category), (int)temp_ptr->primary_oui[0], (int)temp_ptr->primary_oui[1], (int)temp_ptr->primary_oui[2], (int)temp_ptr->primary_oui[3], (int)ntohs(temp_ptr->primary_subcategory));
						printf("\t Group WifiDirect Client Secondary Device Count = %d\n", (int)temp_ptr->wifidirect_secondary_dev_count);
						array_ptr =
							temp_ptr->
							wifidirect_secondary_dev_info;
						for (i = 0;
						     i <
						     temp_ptr->
						     wifidirect_secondary_dev_count;
						     i++) {
							memcpy(&temp, array_ptr,
							       sizeof(t_u16));
							printf("\t Group WifiDirect Client Secondary device type = %02d-", ntohs(temp));
							array_ptr +=
								sizeof(temp);
							printf("%02X%02X%02X%02X", array_ptr[0], array_ptr[1], array_ptr[2], array_ptr[3]);
							array_ptr += 4;
							memcpy(&temp, array_ptr,
							       sizeof(t_u16));
							printf("-%02d\n",
							       ntohs(temp));
							array_ptr +=
								sizeof(temp);
						}
						/* display device name */
						array_ptr =
							temp_ptr->
							wifidirect_device_name +
							temp_ptr->
							wifidirect_secondary_dev_count
							* WPS_DEVICE_TYPE_LEN;
						printf("\t Group WifiDirect Device Name =  ");
						memcpy(&temp,
						       (((t_u8 *)(&temp_ptr->
								  wifidirect_device_name_len))
							+
							temp_ptr->
							wifidirect_secondary_dev_count
							* WPS_DEVICE_TYPE_LEN),
						       sizeof(t_u16));
						temp = ntohs(temp);
						for (i = 0; i < temp; i++)
							printf("%c",
							       *array_ptr++);
						printf("\n");
						temp_ptr +=
							wifidirect_client_dev_length;
						temp -= wifidirect_client_dev_length;
						if (temp_ptr)
							wifidirect_client_dev_length
								=
								temp_ptr->
								dev_length;
					}
					printf("\n");
				}
				break;
			case TLV_TYPE_WIFIDIRECT_GROUP_ID:
				{
					tlvbuf_wifidirect_group_id
						*wifidirect_tlv =
						(tlvbuf_wifidirect_group_id *)
						ptr;
					printf("\t Group address - ");
					print_mac(wifidirect_tlv->
						  group_address);
					printf("\n");
					printf("\t Group ssid =  ");
					for (i = 0;
					     (unsigned int)i <
					     uap_le16_to_cpu(wifidirect_tlv->
							     length)
					     -
					     (sizeof(tlvbuf_wifidirect_group_id)
					      - WIFIDIRECT_IE_HEADER_LEN); i++)
						printf("%c",
						       wifidirect_tlv->
						       group_ssid[i]);
					printf("\n");
				}
				break;
			case TLV_TYPE_WIFIDIRECT_GROUP_BSS_ID:
				{
					tlvbuf_wifidirect_group_bss_id
						*wifidirect_tlv =
						(tlvbuf_wifidirect_group_bss_id
						 *)ptr;
					printf("\t Group BSS Id - ");
					print_mac(wifidirect_tlv->group_bssid);
					printf("\n");
				}
				break;
			case TLV_TYPE_WIFIDIRECT_INTERFACE:
				{
					tlvbuf_wifidirect_interface
						*wifidirect_tlv =
						(tlvbuf_wifidirect_interface *)
						ptr;
					printf("\t Interface Id - ");
					print_mac(wifidirect_tlv->interface_id);
					printf("\t Interface count = %d",
					       (int)wifidirect_tlv->
					       interface_count);
					for (i = 0;
					     i <
					     wifidirect_tlv->interface_count;
					     i++) {
						printf("\n\t Interface address [%d]", i + 1);
						print_mac(&wifidirect_tlv->
							  interface_idlist[i *
									   ETH_ALEN]);
					}
					printf("\n");
				}
				break;
			case TLV_TYPE_WIFIDIRECT_CHANNEL:
				{
					tlvbuf_wifidirect_channel
						*wifidirect_tlv =
						(tlvbuf_wifidirect_channel *)
						ptr;
					printf("\t Listen Channel Country String %c%c", wifidirect_tlv->country_string[0], wifidirect_tlv->country_string[1]);
					if (isalpha
					    (wifidirect_tlv->country_string[2]))
						printf("%c",
						       wifidirect_tlv->
						       country_string[2]);
					printf("\n");
					printf("\t Listen Channel regulatory class = %d\n", (int)wifidirect_tlv->regulatory_class);
					printf("\t Listen Channel number = %d\n", (int)wifidirect_tlv->channel_number);
				}
				break;

			case TLV_TYPE_WIFIDIRECT_OPCHANNEL:
				{
					tlvbuf_wifidirect_channel
						*wifidirect_tlv =
						(tlvbuf_wifidirect_channel *)
						ptr;
					printf("\t Operating Channel Country String %c%c", wifidirect_tlv->country_string[0], wifidirect_tlv->country_string[1]);
					if (isalpha
					    (wifidirect_tlv->country_string[2]))
						printf("%c",
						       wifidirect_tlv->
						       country_string[2]);
					printf("\n");
					printf("\t Operating Channel regulatory class = %d\n", (int)wifidirect_tlv->regulatory_class);
					printf("\t Operating Channel number = %d\n", (int)wifidirect_tlv->channel_number);
				}
				break;

			case TLV_TYPE_WIFIDIRECT_CONFIG_TIMEOUT:
				{
					tlvbuf_wifidirect_config_timeout
						*wifidirect_tlv =
						(tlvbuf_wifidirect_config_timeout
						 *)ptr;
					printf("\t GO configuration timeout = %d msec\n", (int)wifidirect_tlv->group_config_timeout * 10);
					printf("\t Client configuration timeout = %d msec\n", (int)wifidirect_tlv->device_config_timeout * 10);
				}
				break;
			case TLV_TYPE_WIFIDIRECT_EXTENDED_LISTEN_TIME:
				{
					tlvbuf_wifidirect_ext_listen_time
						*wifidirect_tlv =
						(tlvbuf_wifidirect_ext_listen_time
						 *)ptr;
					printf("\t Availability Period = %d msec\n", (int)wifidirect_tlv->availability_period);
					printf("\t Availability Interval = %d msec\n", (int)wifidirect_tlv->availability_interval);
				}
				break;
			case TLV_TYPE_WIFIDIRECT_INTENDED_ADDRESS:
				{
					tlvbuf_wifidirect_intended_addr
						*wifidirect_tlv =
						(tlvbuf_wifidirect_intended_addr
						 *)ptr;
					printf("\t Intended Interface Address - ");
					print_mac(wifidirect_tlv->
						  group_address);
					printf("\n");
				}
				break;

			case TLV_TYPE_WIFIDIRECT_STATUS:
				{
					tlvbuf_wifidirect_status *wifidirect_tlv
						=
						(tlvbuf_wifidirect_status *)ptr;
					printf("\t Status = %d\n",
					       wifidirect_tlv->status_code);
				}
				break;

			default:
				printf("unknown ie=0x%x, len=%d\n", type, len);
				break;
			}
			next_byte = WIFIDIRECT_OVERLAP_TYPE;
			if (orig_ptr)
				ptr = orig_ptr + pending_len;
		}
		ptr += len + WIFIDIRECT_IE_HEADER_LEN;
		left_len -= len;
	}
	printf("\n");
	return;
}

/**
 *  @brief Parse and print WIFIDIRECT generic event data
 *
 *  @param buffer   Pointer to received buffer
 *  @param size     Length of the received event data
 *  @return         N/A
 */
void
print_event_wifidirect_generic(t_u8 *buffer, t_u16 size)
{
	const t_u8 wifi_oui[3] = { 0x50, 0x6F, 0x9A };
	const t_u8 wps_oui[3] = { 0x00, 0x50, 0xF2 };
	apeventbuf_wifidirect_generic *wifidirect_event;
	wifidirect_ie_header *wifidirect_wps_header;
	t_u16 wifidirect_wps_len = 0, type, sub_type;
	printf("EVENT: WIFIDIRECT \n");
	wifidirect_event = (apeventbuf_wifidirect_generic *)(buffer);
	printf("Event length = %d\n",
	       uap_le16_to_cpu(wifidirect_event->event_length));
	printf("Event Type = ");
	type = uap_le16_to_cpu(wifidirect_event->event_type);
	switch (type) {
	case 0:
		printf("Negotiation Request\n");
		break;
	case 1:
		printf("Negotiation Response\n");
		break;
	case 2:
		printf("Negotiation Result\n");
		break;
	case 3:
		printf("Invitation Request\n");
		break;
	case 4:
		printf("Invitation Response\n");
		break;
	case 5:
		printf("Discoverability Request\n");
		break;
	case 6:
		printf("Discoverability Response\n");
		break;
	case 7:
		printf("Provision Discovery Request\n");
		break;
	case 8:
		printf("Provision Discovery Response\n");
		break;
	case 9:
		printf("GO Negotiation response Tx Event\n");
		break;
	case 10:
		printf("GO Negotiation confirm Tx Event\n");
		break;
	case 14:
		printf("Peer Detected event\n");
		break;
	case 15:
		printf("Client associated event\n");
		break;
	case 16:
		printf("FW debug event: %s\n",
		       wifidirect_event->entire_ie_list);
		return;
	default:
		printf("Unknown\n");
		break;
	}
	sub_type = uap_le16_to_cpu(wifidirect_event->event_sub_type);
	if (type == 2) {
		switch (sub_type) {
		case 0:
			printf("Event SubType = No Role\n");
			break;
		case 1:
			printf("Event SubType = Group Owner Role\n");
			break;
		case 2:
			printf("Event SubType = Client Role\n");
			break;
		default:
			printf("Event SubType = %d\n", sub_type);
			break;
		}
	} else if (type == 3 || type == 4) {
		switch (sub_type & 0x03) {	/* lower 2 bits */
		case 0:
			printf("Event SubType = No Role\n");
			break;
		case 1:
			printf("Event SubType = Group Owner Role\n");
			break;
		case 2:
			printf("Event SubType = Client Role\n");
			break;
		}
		switch ((sub_type & 0x1C) >> 2) {	/* next 3 bits */
		case 0:
			printf("Packet processing state = None\n");
			break;
		case 1:
			printf("Packet processing state = Processing\n");
			break;
		case 2:
			printf("Packet processing state = Insufficient information, Dropped\n");
			break;
		case 3:
			printf("Packet processing state = Success\n");
			break;
		case 4:
			printf("Packet processing state = Fail\n");
			break;
		default:
			printf("Event SubType = %d\n", sub_type);
			break;
		}
	} else if (type == 7 || type == 8) {
		switch (sub_type) {
		case 0:
			printf("Event SubType = No Config Method\n");
			break;
		case 8:
			printf("Event SubType = Config Method Display\n");
			break;
		case 0x80:
			printf("Event SubType = Config Method Push Button\n");
			break;
		case 0x100:
			printf("Event SubType = Config Method Keypad\n");
			break;
		default:
			printf("Event SubType = %d\n", sub_type);
			break;
		}
	}
	printf("Peer Mac Address - ");
	print_mac(wifidirect_event->peer_mac_addr);
	printf("\n");
	/* Print rest of IE elements */
	wifidirect_wps_header =
		(wifidirect_ie_header *)(wifidirect_event->entire_ie_list);
	wifidirect_wps_len = uap_le16_to_cpu(wifidirect_event->event_length)
		- sizeof(apeventbuf_wifidirect_generic);

	while (wifidirect_wps_len >= sizeof(wifidirect_ie_header)) {
		if (!memcmp
		    (wifidirect_wps_header->oui, wifi_oui, sizeof(wifi_oui)) ||
		    !(memcmp
		      (wifidirect_wps_header->oui, wps_oui, sizeof(wps_oui)))) {
			switch (wifidirect_wps_header->oui_type) {
			case WIFIDIRECT_OUI_TYPE:
				print_wifidirect_ie_elements
					(wifidirect_wps_header->ie_list,
					 wifidirect_wps_header->ie_length -
					 sizeof(wifidirect_wps_header->oui)
					 -
					 sizeof(wifidirect_wps_header->
						oui_type));
				printf("\n");
				break;
			case WIFI_WPS_OUI_TYPE:
				print_wifi_wps_ie_elements
					(wifidirect_wps_header->ie_list,
					 wifidirect_wps_header->ie_length -
					 sizeof(wifidirect_wps_header->oui)
					 -
					 sizeof(wifidirect_wps_header->
						oui_type));
				printf("\n");
				break;
			}
		}
		wifidirect_wps_len -=
			wifidirect_wps_header->ie_length + IE_HEADER_LEN;
		wifidirect_wps_header =
			(wifidirect_ie_header *)(((t_u8 *)wifidirect_wps_header)
						 +
						 wifidirect_wps_header->
						 ie_length + IE_HEADER_LEN);
	}
}

/**
 *  @brief Parse and print WIFIDIRECT service discovery event data
 *
 *  @param buffer   Pointer to received buffer
 *  @param size     Length of the received event data
 *  @return         N/A
 */
void
print_event_wifidirect_service_discovery(t_u8 *buffer, t_u16 size)
{
	unsigned int i;
	t_u16 event_len = 0;
	t_u16 dns_len = 0, dns_type;
	t_u8 action = 0;	/* req = 0, resp = 1 */
	apeventbuf_wifidirect_discovery_request *wifidirect_disc_req;
	apeventbuf_wifidirect_discovery_response *wifidirect_disc_resp;
	printf("EVENT: WIFIDIRECT SERVICE DISCOVERY\n");
	memcpy(&event_len, buffer, sizeof(t_u16));
	event_len = uap_le16_to_cpu(event_len);
	printf("Event length = %d\n", event_len);
	printf("Service Discovery packet:\n");
	/* check request /response Byte at offset 2+6+2 */
	action = *(buffer + sizeof(t_u16) + ETH_ALEN + sizeof(t_u8));
	if (action == WIFIDIRECT_DISCOVERY_REQUEST_ACTION) {
		wifidirect_disc_req =
			(apeventbuf_wifidirect_discovery_request *)(buffer +
								    sizeof
								    (t_u16));
		printf("\t Peer Mac Address - ");
		print_mac(wifidirect_disc_req->peer_mac_addr);
		printf("\n\t Category = %d\n", wifidirect_disc_req->category);
		printf("\t Action = %d\n", wifidirect_disc_req->action);
		printf("\t Dialog taken = %d\n",
		       wifidirect_disc_req->dialog_taken);
		printf("\t Advertize protocol IE - 0x%02x, 0x%02x, 0x%02x, 0x%02x\n", wifidirect_disc_req->advertize_protocol_ie[0], wifidirect_disc_req->advertize_protocol_ie[1], wifidirect_disc_req->advertize_protocol_ie[2], wifidirect_disc_req->advertize_protocol_ie[3]);
		printf("\t Request query length = %d\n",
		       uap_le16_to_cpu(wifidirect_disc_req->query_len));
		printf("\t Information Id - 0x%02x, 0x%02x\n",
		       wifidirect_disc_req->info_id[0],
		       wifidirect_disc_req->info_id[1]);
		printf("\t Request length = %d\n",
		       uap_le16_to_cpu(wifidirect_disc_req->request_len));
		printf("\t OUI - 0x%02x, 0x%02x, 0x%02x\n",
		       wifidirect_disc_req->oui[0], wifidirect_disc_req->oui[1],
		       wifidirect_disc_req->oui[2]);
		printf("\t OUI sub type = %d\n",
		       wifidirect_disc_req->oui_sub_type);
		printf("\t Service update Indicator = %d\n",
		       uap_le16_to_cpu(wifidirect_disc_req->
				       service_update_indicator));
		printf("\t Vendor length = %d\n",
		       uap_le16_to_cpu(wifidirect_disc_req->vendor_len));
		printf("\t Service protocol = %d\n",
		       wifidirect_disc_req->service_protocol);
		printf("\t Service transaction Id = %d\n",
		       wifidirect_disc_req->service_transaction_id);
		printf("\t Query Data = ");
		if (wifidirect_disc_req->service_protocol == 1) {
			printf(" * Bonjour * \n");
			printf("\t\t DNS = ");
			dns_len =
				uap_le16_to_cpu(wifidirect_disc_req->
						vendor_len) -
				WIFIDIRECT_DISCOVERY_BONJOUR_FIXED_LEN;
			for (i = 0; i < dns_len; i++)
				printf("%02x ",
				       (int)*(wifidirect_disc_req->disc_query.u.
					      bonjour.dns + i));
			memcpy(&dns_type,
			       (&wifidirect_disc_req->disc_query.u.bonjour.
				dns_type + dns_len), sizeof(dns_type));
			dns_type = uap_le16_to_cpu(dns_type);
			printf("\n\t\t DNS Type = %d\n", dns_type);
			printf("\t\t Version = %d\n",
			       *(&wifidirect_disc_req->disc_query.u.bonjour.
				 version + dns_len));
		} else if (wifidirect_disc_req->service_protocol == 2) {
			printf(" * uPnP * \n");
			printf("\t\t Version = %d\n",
			       wifidirect_disc_req->disc_query.u.upnp.version);
			dns_len =
				uap_le16_to_cpu(wifidirect_disc_req->
						vendor_len) -
				WIFIDIRECT_DISCOVERY_UPNP_FIXED_LEN;
			printf("\t\t Value = ");
			for (i = 0; i < dns_len; i++)
				printf("%02x ",
				       (int)*(wifidirect_disc_req->disc_query.u.
					      upnp.value + i));
		}
		printf("\n");
	} else if (action == WIFIDIRECT_DISCOVERY_RESPONSE_ACTION) {
		wifidirect_disc_resp =
			(apeventbuf_wifidirect_discovery_response *)(buffer +
								     sizeof
								     (t_u16));
		printf("\t Peer Mac Address - ");
		print_mac(wifidirect_disc_resp->peer_mac_addr);
		printf("\n\t Category = %d\n", wifidirect_disc_resp->category);
		printf("\t Action = %d\n", wifidirect_disc_resp->action);
		printf("\t Dialog taken = %d\n",
		       wifidirect_disc_resp->dialog_taken);
		printf("\t Status code = %d\n",
		       wifidirect_disc_resp->status_code);
		printf("\t GAS reply - 0x%02x\n",
		       uap_le16_to_cpu(wifidirect_disc_resp->gas_reply));
		printf("\t Advertize protocol IE - 0x%02x, 0x%02x, 0x%02x, 0x%02x\n", wifidirect_disc_resp->advertize_protocol_ie[0], wifidirect_disc_resp->advertize_protocol_ie[1], wifidirect_disc_resp->advertize_protocol_ie[2], wifidirect_disc_resp->advertize_protocol_ie[3]);
		printf("\t Response query length = %d\n",
		       uap_le16_to_cpu(wifidirect_disc_resp->query_len));
		printf("\t Information Id - 0x%02x, 0x%02x\n",
		       wifidirect_disc_resp->info_id[0],
		       wifidirect_disc_resp->info_id[1]);
		printf("\t Response length = %d\n",
		       uap_le16_to_cpu(wifidirect_disc_resp->response_len));
		printf("\t OUI - 0x%02x, 0x%02x, 0x%02x\n",
		       wifidirect_disc_resp->oui[0],
		       wifidirect_disc_resp->oui[1],
		       wifidirect_disc_resp->oui[2]);
		printf("\t OUI sub type = %d\n",
		       wifidirect_disc_resp->oui_sub_type);
		printf("\t Service update Indicator = %d\n",
		       uap_le16_to_cpu(wifidirect_disc_resp->
				       service_update_indicator));
		printf("\t Vendor length = %d\n",
		       uap_le16_to_cpu(wifidirect_disc_resp->vendor_len));
		printf("\t Service protocol = %d\n",
		       wifidirect_disc_resp->service_protocol);
		printf("\t Service transaction Id = %d\n",
		       wifidirect_disc_resp->service_transaction_id);
		printf("\t Status Code = %d\n",
		       wifidirect_disc_resp->disc_status_code);
		printf("\t Response Data = ");
		if (wifidirect_disc_resp->service_protocol == 1) {
			printf(" * Bonjour * \n");
			/*
			   printf("\t\t DNS = ");
			   dns_len = uap_le16_to_cpu(wifidirect_disc_resp->vendor_len) -
			   (WIFIDIRECT_DISCOVERY_BONJOUR_FIXED_LEN + 1);
			   for( i=0; i < dns_len; i++)
			   printf("%c",*(wifidirect_disc_resp->disc_resp.u.bonjour.dns + i));
			   memcpy(&dns_type, (&wifidirect_disc_req->disc_query.u.bonjour.dns_type
			   + dns_len), sizeof(dns_type));
			   dns_type = uap_le16_to_cpu(dns_type);
			   printf("\n\t\t DNS Type = %d\n", dns_type);
			   printf("\t\t Version = %d\n", *(&wifidirect_disc_resp->disc_resp.u.bonjour.version
			   + dns_len));
			 */
		} else if (wifidirect_disc_resp->service_protocol == 2) {
			printf(" * uPnP * \n");
			printf("\t\t Version = %d\n",
			       wifidirect_disc_resp->disc_resp.u.upnp.version);
			dns_len =
				uap_le16_to_cpu(wifidirect_disc_resp->
						vendor_len) -
				WIFIDIRECT_DISCOVERY_UPNP_FIXED_LEN;
			printf("\t\t Value = ");
			for (i = 0; i < dns_len; i++)
				printf("%02x ",
				       (int)*(wifidirect_disc_resp->disc_resp.u.
					      upnp.value + i));
		}
		printf("\n");
	}
}
#endif

/**
 *  @brief Parse and print TDLS generic event data
 *
 *  @param buffer   Pointer to received buffer
 *  @param size     Length of the received event data
 *  @return         N/A
 */
void
print_event_tdls_generic(t_u8 *buffer, t_u16 size)
{
	eventbuf_tdls_generic *tdls_event;
	eventbuf_tdls_debug *tdls_debug;
	eventbuf_tdls_packet *tdls_pkt;
	printf("EVENT: TDLS\n");
	tdls_event = (eventbuf_tdls_generic *)(buffer);
	printf("Event Type = ");
	switch (uap_le16_to_cpu(tdls_event->event_type)) {
	case TDLS_EVENT_TYPE_SETUP_FAILURE:
		printf("TDLS setup failure\n");
		break;
	case TDLS_EVENT_TYPE_SETUP_REQ_RCVD:
		printf("TDLS setup request received from peer\n");
		break;
	case TDLS_EVENT_TYPE_LINK_TORN_DOWN:
		printf("TDLS link torn down\n");
		break;
	case TDLS_EVENT_TYPE_LINK_ESTABLISHED:
		printf("TDLS link established\n");
		break;
	case TDLS_EVENT_TYPE_DEBUG:
		tdls_debug = (eventbuf_tdls_debug *)(buffer);
		printf("TDLS debug event, ");
		printf("TSF  = %lld\n", uap_le64_to_cpu(tdls_debug->tsf));
		printf("String = ");
		printf((char *)(tdls_debug->string),
		       uap_le32_to_cpu(tdls_debug->first_arg),
		       uap_le32_to_cpu(tdls_debug->second_arg),
		       uap_le32_to_cpu(tdls_debug->third_arg));
		printf("\n");
		return;
	case TDLS_EVENT_TYPE_PACKET:
		tdls_pkt = (eventbuf_tdls_packet *)(buffer);
		printf("TDLS packet len=%d\n", tdls_pkt->length);
		hexdump((t_u8 *)tdls_pkt->data, tdls_pkt->length, ' ');
		return;
	case TDLS_EVENT_TYPE_CHAN_SWITCH_RESULT:
		if (tdls_event->u.switch_result.status) {
			printf("TDLS Channel Switch Failed! current channel=%d Reasoncode %d\n", tdls_event->u.switch_result.current_channel, tdls_event->u.switch_result.reason);
		} else {
			printf("TDLS Channel Switch Successful. current channel=%d\n", tdls_event->u.switch_result.current_channel);
		}
		break;
	case TDLS_EVENT_TYPE_START_CHAN_SWITCH:
		printf("TDLS Start channel switch...\n");
		break;
	case TDLS_EVENT_TYPE_CHAN_SWITCH_STOPPED:
		printf("TDLS Channel Switch stopped! reason=%d\n",
		       tdls_event->u.cs_stop_reason);
		break;
	default:
		printf("Unknown\n");
		break;
	}
	printf("Peer Mac Address - ");
	print_mac(tdls_event->peer_mac_addr);
	printf("\n");
	if (uap_le16_to_cpu(tdls_event->event_type) ==
	    TDLS_EVENT_TYPE_SETUP_FAILURE) {
		switch (uap_le16_to_cpu(tdls_event->u.reason_code)) {
		case 1:
			printf("Reason Code = Internal error\n");
			break;
		case 7:
			printf("Reason Code = Timeout waiting for response\n");
			break;
		case 8:
			printf("Reason Code = Non-zero setup response status\n");
			break;
		default:
			printf("Reason Code = %d\n",
			       uap_le16_to_cpu(tdls_event->u.reason_code));
			break;
		}
	} else if (uap_le16_to_cpu(tdls_event->event_type) ==
		   TDLS_EVENT_TYPE_LINK_TORN_DOWN) {
		switch (uap_le16_to_cpu(tdls_event->u.reason_code)) {
		case 3:
			printf("Reason Code = Peer station is leaving BSS\n");
			break;
		case 25:
			printf("Reason Code = Peer station is unreachable on direct link\n");
			break;
		case 26:
			printf("Reason Code = Unspecified\n");
			break;
		case 211:
			printf("Reason Code = Setup received from TDLS peer\n");
			break;
		case 212:
			printf("Reason Code = Clearing TDLS connections due to deauth/disassoc\n");
			break;
		default:
			printf("Reason Code = %d\n",
			       uap_le16_to_cpu(tdls_event->u.reason_code));
			break;
		}
	} else if (uap_le16_to_cpu(tdls_event->event_type) ==
		   TDLS_EVENT_TYPE_LINK_ESTABLISHED) {
		int ie_len = uap_le16_to_cpu(tdls_event->u.ie_length);
		if (ie_len) {
			printf("Peer IE len=%d\n", ie_len);
			hexdump((t_u8 *)tdls_event->ie_ptr, ie_len, ' ');
		}
	}
}

/**
 *  @brief Prints station reject state
 *
 *  @param state    Fail state
 *  @return         N/A
 */
void
print_reject_state(t_u8 state)
{
	switch (state) {
	case REJECT_STATE_FAIL_EAPOL_2:
		printf("Reject state: FAIL_EAPOL_2\n");
		break;
	case REJECT_STATE_FAIL_EAPOL_4:
		printf("Reject state: FAIL_EAPOL_4:\n");
		break;
	case REJECT_STATE_FAIL_EAPOL_GROUP_2:
		printf("Reject state: FAIL_EAPOL_GROUP_2\n");
		break;
	default:
		printf("ERR: unknown reject state %d\n", state);
		break;
	}
	return;
}

/**
 *  @brief Prints station reject reason
 *
 *  @param reason   Reason code
 *  @return         N/A
 */
void
print_reject_reason(t_u16 reason)
{
	switch (reason) {
	case IEEEtypes_REASON_INVALID_IE:
		printf("Reject reason: Invalid IE\n");
		break;
	case IEEEtypes_REASON_MIC_FAILURE:
		printf("Reject reason: Mic Failure\n");
		break;
	default:
		printf("Reject reason: %d\n", reason);
		break;
	}
	return;
}

/**
 *  @brief Prints EAPOL state
 *
 *  @param state    Eapol state
 *  @return         N/A
 */
void
print_eapol_state(t_u8 state)
{
	switch (state) {
	case EAPOL_START:
		printf("Eapol state: EAPOL_START\n");
		break;
	case EAPOL_WAIT_PWK2:
		printf("Eapol state: EAPOL_WAIT_PWK2\n");
		break;
	case EAPOL_WAIT_PWK4:
		printf("Eapol state: EAPOL_WAIT_PWK4\n");
		break;
	case EAPOL_WAIT_GTK2:
		printf("Eapol state: EAPOL_WAIT_GTK2\n");
		break;
	case EAPOL_END:
		printf("Eapol state: EAPOL_END\n");
		break;
	default:
		printf("ERR: unknow eapol state%d\n", state);
		break;
	}
	return;
}

/**
 *  @brief Parse and print debug event data
 *
 *  @param buffer   Pointer to received buffer
 *  @param size     Length of the received event data
 *  @return         N/A
 */
void
print_event_debug(t_u8 *buffer, t_u16 size)
{
	eventbuf_debug *event_body = NULL;
	if (size < sizeof(eventbuf_debug)) {
		printf("ERR:Event buffer too small!\n");
		return;
	}
	event_body = (eventbuf_debug *)buffer;
	printf("Debug Event Type: %s\n",
	       (event_body->debug_type == 0) ? "EVENT" : "INFO");
	printf("%s log:\n",
	       (uap_le32_to_cpu(event_body->debug_id_major) ==
		DEBUG_ID_MAJ_AUTHENTICATOR) ? "Authenticator" : "Assoc_agent");
	if (uap_le32_to_cpu(event_body->debug_id_major) ==
	    DEBUG_ID_MAJ_AUTHENTICATOR) {
		switch (uap_le32_to_cpu(event_body->debug_id_minor)) {
		case DEBUG_MAJ_AUTH_MIN_PWK1:
			printf("EAPOL Key message 1 (PWK):\n");
			hexdump((t_u8 *)&event_body->info.eapol_pwkmsg,
				sizeof(eapol_keymsg_debug_t), ' ');
			break;
		case DEBUG_MAJ_AUTH_MIN_PWK2:
			printf("EAPOL Key message 2 (PWK):\n");
			hexdump((t_u8 *)&event_body->info.eapol_pwkmsg,
				sizeof(eapol_keymsg_debug_t), ' ');
			break;
		case DEBUG_MAJ_AUTH_MIN_PWK3:
			printf("EAPOL Key message 3 (PWK):\n");
			hexdump((t_u8 *)&event_body->info.eapol_pwkmsg,
				sizeof(eapol_keymsg_debug_t), ' ');
			break;
		case DEBUG_MAJ_AUTH_MIN_PWK4:
			printf("EAPOL Key message 4: (PWK)\n");
			hexdump((t_u8 *)&event_body->info.eapol_pwkmsg,
				sizeof(eapol_keymsg_debug_t), ' ');
			break;
		case DEBUG_MAJ_AUTH_MIN_GWK1:
			printf("EAPOL Key message 1: (GWK)\n");
			hexdump((t_u8 *)&event_body->info.eapol_pwkmsg,
				sizeof(eapol_keymsg_debug_t), ' ');
			break;
		case DEBUG_MAJ_AUTH_MIN_GWK2:
			printf("EAPOL Key message 2: (GWK)\n");
			hexdump((t_u8 *)&event_body->info.eapol_pwkmsg,
				sizeof(eapol_keymsg_debug_t), ' ');
			break;
		case DEBUG_MAJ_AUTH_MIN_STA_REJ:
			printf("Reject STA MAC: ");
			print_mac(event_body->info.sta_reject.sta_mac_addr);
			printf("\n");
			print_reject_state(event_body->info.sta_reject.
					   reject_state);
			print_reject_reason(uap_le16_to_cpu
					    (event_body->info.sta_reject.
					     reject_reason));
			break;
		case DEBUG_MAJ_AUTH_MIN_EAPOL_TR:
			printf("STA MAC: ");
			print_mac(event_body->info.eapol_state.sta_mac_addr);
			printf("\n");
			print_eapol_state(event_body->info.eapol_state.
					  eapol_state);
			break;
		default:
			printf("ERR: unknow debug_id_minor: %d\n",
			       (int)uap_le32_to_cpu(event_body->
						    debug_id_minor));
			hexdump(buffer, size, ' ');
			return;
		}
	} else if (uap_le32_to_cpu(event_body->debug_id_major) ==
		   DEBUG_ID_MAJ_ASSOC_AGENT) {
		switch (uap_le32_to_cpu(event_body->debug_id_minor)) {
		case DEBUG_ID_MAJ_ASSOC_MIN_WPA_IE:
			printf("STA MAC: ");
			print_mac(event_body->info.wpaie.sta_mac_addr);
			printf("\n");
			printf("wpa ie:\n");
			hexdump(event_body->info.wpaie.wpa_ie, MAX_WPA_IE_LEN,
				' ');
			break;
		case DEBUG_ID_MAJ_ASSOC_MIN_STA_REJ:
			printf("Reject STA MAC: ");
			print_mac(event_body->info.sta_reject.sta_mac_addr);
			printf("\n");
			print_reject_state(event_body->info.sta_reject.
					   reject_state);
			print_reject_reason(uap_le16_to_cpu
					    (event_body->info.sta_reject.
					     reject_reason));
			break;
		default:
			printf("ERR: unknow debug_id_minor: %d\n",
			       (int)uap_le32_to_cpu(event_body->
						    debug_id_minor));
			hexdump(buffer, size, ' ');
			return;
		}
	}
	return;
}

/**
 *  @brief Parse and print received nlist event information
 *
 *  @param buffer   Pointer to the data buffer
 *  @param size     Length of the received event
 *  @return         N/A
 */
void
print_event_11k_nlist_report(t_u8 *buffer, t_u16 size)
{
	int tlv_buf_left = 0;
	tlvbuf_header *tlv;
	nlist_entry_tlv *nlist;
	int entry_num = 0;
	t_u16 tlv_type, tlv_len;

	if (size < 2) {
		printf("No neighbor AP list found: %d\n", size);
		return;
	}
	tlv_buf_left = *(t_u16 *)buffer;
	tlv_buf_left = uap_le16_to_cpu(tlv_buf_left);
	if (tlv_buf_left < (int)sizeof(tlvbuf_header))
		return;
	printf("neighbor AP list tlv len: %d\n", tlv_buf_left);
	tlv = (tlvbuf_header *)(buffer + 2);
	while (tlv_buf_left >= (int)sizeof(tlvbuf_header)) {
		tlv_type = uap_le16_to_cpu(tlv->type);
		tlv_len = uap_le16_to_cpu(tlv->len);
		if ((sizeof(tlvbuf_header) + tlv_len) >
		    (unsigned int)tlv_buf_left) {
			printf("wrong tlv: tlvLen=%d, tlvBufLeft=%d\n", tlv_len,
			       tlv_buf_left);
			break;
		}
		switch (tlv_type) {
		case MRVL_NEIGHBOR_REPORT_TLV_ID:
			nlist = (nlist_entry_tlv *) tlv;
			printf("Neighbor entry %d:\n", entry_num);
			printf("BSSID: %02x:%02x:%02x:%02x:%02x:%02x \n",
			       nlist->bssid[0], nlist->bssid[1],
			       nlist->bssid[2], nlist->bssid[3],
			       nlist->bssid[4], nlist->bssid[5]);
			printf("BSSID Info: %02x:%02x:%02x:%02x \n",
			       nlist->bssid_info[0], nlist->bssid_info[1],
			       nlist->bssid_info[2], nlist->bssid_info[3]);
			printf("reg class: %d, chan: %d, phy: %d \n",
			       nlist->reg_class, nlist->chan, nlist->phy_type);
			if (tlv_len > sizeof(nlist_entry_tlv)) {
				printf("sub IE:\n");
				hexdump((t_u8 *)tlv + sizeof(nlist_entry_tlv),
					tlv_len - sizeof(nlist_entry_tlv) +
					sizeof(tlvbuf_header), ' ');
			}
			entry_num++;
			break;
		default:
#if DEBUG
			mlanevent_printf(MSG_DEBUG, "unknown tlv: %d\n",
					 tlv_type);
#endif
			break;
		}
		tlv_buf_left -= (sizeof(tlvbuf_header) + tlv_len);
		tlv = (tlvbuf_header *)((t_u8 *)tlv + tlv_len +
					sizeof(tlvbuf_header));
	}
	return;
}

/**
 *  @brief Parse and print received event information
 *
 *  @param event    Pointer to received event
 *  @param size     Length of the received event
 *  @return         N/A
 */
void
print_event(event_header *event, t_u16 size)
{
	t_u32 event_id = event->event_id;
	switch (event_id) {
	case MICRO_AP_EV_ID_STA_DEAUTH:
		print_event_sta_deauth(event->event_data, size - EVENT_ID_LEN);
		break;
	case MICRO_AP_EV_ID_STA_ASSOC:
		print_event_sta_assoc(event->event_data, size - EVENT_ID_LEN);
		break;
	case MICRO_AP_EV_ID_BSS_START:
		print_event_bss_start(event->event_data, size - EVENT_ID_LEN);
		break;
	case MICRO_AP_EV_ID_DEBUG:
		print_event_debug(event->event_data, size - EVENT_ID_LEN);
		break;
	case MICRO_AP_EV_BSS_IDLE:
		printf("EVENT: BSS_IDLE\n");
		break;
	case MICRO_AP_EV_BSS_ACTIVE:
		printf("EVENT: BSS_ACTIVE\n");
		break;
	case MICRO_AP_EV_RSN_CONNECT:
		print_event_rsn_connect(event->event_data, size - EVENT_ID_LEN);
		break;
#ifdef WIFI_DIRECT_SUPPORT
	case EVENT_WIFIDIRECT_GENERIC:
		print_event_wifidirect_generic(event->event_data,
					       size - EVENT_ID_LEN);
		break;
	case EVENT_WIFIDIRECT_SERVICE_DISCOVERY:
		print_event_wifidirect_service_discovery(event->event_data,
							 size - EVENT_ID_LEN);
		break;
#endif

	case EVENT_TDLS_GENERIC:
		print_event_tdls_generic(event->event_data,
					 size - EVENT_ID_LEN);
		break;
	case UAP_EVENT_ID_DRV_HS_ACTIVATED:
		printf("EVENT: uAP HS_ACTIVATED\n");
		break;
	case UAP_EVENT_ID_DRV_HS_DEACTIVATED:
		printf("EVENT: uAP HS_DEACTIVATED\n");
		break;
	case UAP_EVENT_ID_HS_WAKEUP:
		printf("EVENT: uAP HS_WAKEUP\n");
		break;
	case UAP_EVENT_HOST_SLEEP_AWAKE:
		break;
	case UAP_EVENT_ID_DRV_MGMT_FRAME:
		printf("EVENT: Mgmt frame from FW\n");
		hexdump((void *)event, size, ' ');
		break;
	case MICRO_AP_EV_WMM_STATUS_CHANGE:
		printf("EVENT: WMM_STATUS_CHANGE\n");
		break;
	case EVENT_RADAR_DETECTED:
		printf("EVENT: RADAR_DETECTED\n");
		break;
	case EVENT_CHANNEL_REPORT_RDY:
		printf("EVENT: CHANNEL_REPORT_READY\n");
		hexdump((void *)event, size, ' ');
		break;
	case EVENT_WEP_ICV_ERROR:
		print_event_wep_icv_error(event->event_data,
					  size - EVENT_ID_LEN);
		break;
	case EVENT_ID_DRV_SCAN_REPORT:
		printf("Scan request completed.\n");
		break;
	case EVENT_NLIST_REPORT:
		printf("EVENT: 11k neighbor AP list report\n");
		print_event_11k_nlist_report(event->event_data,
					     size - EVENT_ID_LEN);
		break;

	default:
		/* Handle string based events */
#define CUS_EVT_PORT_RELEASE           "EVENT=PORT_RELEASE"
		if (!strncmp
		    ((char *)event, CUS_EVT_PORT_RELEASE,
		     strlen(CUS_EVT_PORT_RELEASE))) {
			printf("EVENT: PORT_RELEASE.\n");
			break;
		}
#define CUS_EVT_TDLS_CONNECTED           "EVENT=TDLS_CONNECTED"
		if (!strncmp
		    ((char *)event, CUS_EVT_TDLS_CONNECTED,
		     strlen(CUS_EVT_TDLS_CONNECTED))) {
			printf("EVENT: TDLS_CONNECTED\n");
			print_mac((t_u8 *)event +
				  strlen(CUS_EVT_TDLS_CONNECTED));
			printf("\n");
			break;
		}
#define CUS_EVT_TDLS_TEARDOWN            "EVENT=TDLS_TEARDOWN"
		if (!strncmp
		    ((char *)event, CUS_EVT_TDLS_TEARDOWN,
		     strlen(CUS_EVT_TDLS_TEARDOWN))) {
			printf("EVENT: TDLS_TEARDOWN\n");
			print_mac((t_u8 *)event +
				  strlen(CUS_EVT_TDLS_TEARDOWN));
			printf("\n");
			break;
		}
#define CUS_EVT_STA_CONNECTED           "EVENT=STA_CONNECTED"
		if (!strncmp
		    ((char *)event, CUS_EVT_STA_CONNECTED,
		     strlen(CUS_EVT_STA_CONNECTED))) {
			printf("EVENT: STA_CONNECTED\n");
			print_mac((t_u8 *)event +
				  strlen(CUS_EVT_STA_CONNECTED) + 1);
			printf("\n");
			break;
		}
#define CUS_EVT_STA_DISCONNECTED           "EVENT=STA_DISCONNECTED"
		if (!strncmp
		    ((char *)event, CUS_EVT_STA_DISCONNECTED,
		     strlen(CUS_EVT_STA_DISCONNECTED))) {
			printf("EVENT: STA_DISCONNECTED\n");
			break;
		}
#define CUS_EVT_AP_CONNECTED           "EVENT=AP_CONNECTED"
		if (!strncmp
		    ((char *)event, CUS_EVT_AP_CONNECTED,
		     strlen(CUS_EVT_AP_CONNECTED))) {
			printf("EVENT: AP_CONNECTED\n");
			print_mac((t_u8 *)event + strlen(CUS_EVT_AP_CONNECTED));
			printf("\n");
			break;
		}
#define CUS_EVT_ADHOC_LINK_SENSED   "EVENT=ADHOC_LINK_SENSED"
		if (!strncmp
		    ((char *)event, CUS_EVT_ADHOC_LINK_SENSED,
		     strlen(CUS_EVT_ADHOC_LINK_SENSED))) {
			printf("EVENT: ADHOC_LINK_SENSED\n");
			break;
		}
#define CUS_EVT_ADHOC_LINK_LOST     "EVENT=ADHOC_LINK_LOST"
		if (!strncmp
		    ((char *)event, CUS_EVT_ADHOC_LINK_LOST,
		     strlen(CUS_EVT_ADHOC_LINK_LOST))) {
			printf("EVENT: ADHOC_LINK_LOST\n");
			break;
		}
#define CUS_EVT_OBSS_SCAN_PARAM     "EVENT=OBSS_SCAN_PARAM"
		if (!strncmp
		    ((char *)event, CUS_EVT_OBSS_SCAN_PARAM,
		     strlen(CUS_EVT_OBSS_SCAN_PARAM))) {
			printf("EVENT: OBSS_SCAN_PARAM\n");
			break;
		}
#define CUS_EVT_BW_CHANGED      "EVENT=BW_CHANGED"
		if (!strncmp
		    ((char *)event, CUS_EVT_BW_CHANGED,
		     strlen(CUS_EVT_BW_CHANGED))) {
			printf("EVENT: BW_CHANGED\n");
			break;
		}
#define CUS_EVT_MLME_MIC_ERR_UNI    "MLME-MICHAELMICFAILURE.indication unicast"
		if (!strncmp
		    ((char *)event, CUS_EVT_MLME_MIC_ERR_UNI,
		     strlen(CUS_EVT_MLME_MIC_ERR_UNI))) {
			printf("EVENT: MLME-MICHAELMICFAILURE.indication unicast\n");
			break;
		}
#define CUS_EVT_MLME_MIC_ERR_MUL    "MLME-MICHAELMICFAILURE.indication multicast"
		if (!strncmp
		    ((char *)event, CUS_EVT_MLME_MIC_ERR_MUL,
		     strlen(CUS_EVT_MLME_MIC_ERR_MUL))) {
			printf("EVENT: MLME-MICHAELMICFAILURE.indication multicast\n");
			break;
		}
#define CUS_EVT_BEACON_RSSI_LOW     "EVENT=BEACON_RSSI_LOW"
		if (!strncmp
		    ((char *)event, CUS_EVT_BEACON_RSSI_LOW,
		     strlen(CUS_EVT_BEACON_RSSI_LOW))) {
			printf("EVENT: BEACON_RSSI_LOW\n");
			break;
		}
#define CUS_EVT_BEACON_RSSI_HIGH    "EVENT=BEACON_RSSI_HIGH"
		if (!strncmp
		    ((char *)event, CUS_EVT_BEACON_RSSI_HIGH,
		     strlen(CUS_EVT_BEACON_RSSI_HIGH))) {
			printf("EVENT: BEACON_RSSI_HIGH\n");
			break;
		}
#define CUS_EVT_BEACON_SNR_LOW      "EVENT=BEACON_SNR_LOW"
		if (!strncmp
		    ((char *)event, CUS_EVT_BEACON_SNR_LOW,
		     strlen(CUS_EVT_BEACON_SNR_LOW))) {
			printf("EVENT: BEACON_SNR_LOW\n");
			break;
		}
#define CUS_EVT_BEACON_SNR_HIGH     "EVENT=BEACON_SNR_HIGH"
		if (!strncmp
		    ((char *)event, CUS_EVT_BEACON_SNR_HIGH,
		     strlen(CUS_EVT_BEACON_SNR_HIGH))) {
			printf("EVENT: BEACON_SNR_HIGH\n");
			break;
		}
#define CUS_EVT_MAX_FAIL        "EVENT=MAX_FAIL"
		if (!strncmp
		    ((char *)event, CUS_EVT_MAX_FAIL,
		     strlen(CUS_EVT_MAX_FAIL))) {
			printf("EVENT: MAX_FAIL\n");
			break;
		}
#define CUS_EVT_DATA_RSSI_LOW       "EVENT=DATA_RSSI_LOW"
		if (!strncmp
		    ((char *)event, CUS_EVT_DATA_RSSI_LOW,
		     strlen(CUS_EVT_DATA_RSSI_LOW))) {
			printf("EVENT: DATA_RSSI_LOW\n");
			break;
		}
#define CUS_EVT_DATA_SNR_LOW        "EVENT=DATA_SNR_LOW"
		if (!strncmp
		    ((char *)event, CUS_EVT_DATA_SNR_LOW,
		     strlen(CUS_EVT_DATA_SNR_LOW))) {
			printf("EVENT: DATA_SNR_LOW\n");
			break;
		}
#define CUS_EVT_DATA_RSSI_HIGH      "EVENT=DATA_RSSI_HIGH"
		if (!strncmp
		    ((char *)event, CUS_EVT_DATA_RSSI_HIGH,
		     strlen(CUS_EVT_DATA_RSSI_HIGH))) {
			printf("EVENT: DATA_RSSI_HIGH\n");
			break;
		}
#define CUS_EVT_DATA_SNR_HIGH       "EVENT=DATA_SNR_HIGH"
		if (!strncmp
		    ((char *)event, CUS_EVT_DATA_SNR_HIGH,
		     strlen(CUS_EVT_DATA_SNR_HIGH))) {
			printf("EVENT: DATA_SNR_HIGH\n");
			break;
		}
#define CUS_EVT_LINK_QUALITY        "EVENT=LINK_QUALITY"
		if (!strncmp
		    ((char *)event, CUS_EVT_LINK_QUALITY,
		     strlen(CUS_EVT_LINK_QUALITY))) {
			printf("EVENT: LINK_QUALITY\n");
			break;
		}
#define CUS_EVT_WEP_ICV_ERR     "EVENT=WEP_ICV_ERR"
		if (!strncmp
		    ((char *)event, CUS_EVT_WEP_ICV_ERR,
		     strlen(CUS_EVT_WEP_ICV_ERR))) {
			printf("EVENT: WEP_ICV_ERR\n");
			break;
		}
#define CUS_EVT_CHANNEL_SWITCH_ANN  "EVENT=CHANNEL_SWITCH_ANN"
		if (!strncmp
		    ((char *)event, CUS_EVT_CHANNEL_SWITCH_ANN,
		     strlen(CUS_EVT_CHANNEL_SWITCH_ANN))) {
			printf("EVENT: CHANNEL_SWITCH_ANN\n");
			break;
		}
#define CUS_EVT_HS_WAKEUP       "HS_WAKEUP"
		if (!strncmp
		    ((char *)event, CUS_EVT_HS_WAKEUP,
		     strlen(CUS_EVT_HS_WAKEUP))) {
			printf("EVENT: HS_WAKEUP\n");
			break;
		}
#define CUS_EVT_HS_ACTIVATED        "HS_ACTIVATED"
		if (!strncmp
		    ((char *)event, CUS_EVT_HS_ACTIVATED,
		     strlen(CUS_EVT_HS_ACTIVATED))) {
			printf("EVENT: HS_ACTIVATED\n");
			break;
		}
#define CUS_EVT_HS_DEACTIVATED      "HS_DEACTIVATED"
		if (!strncmp
		    ((char *)event, CUS_EVT_HS_DEACTIVATED,
		     strlen(CUS_EVT_HS_DEACTIVATED))) {
			printf("EVENT: HS_DEACTIVATED\n");
			break;
		}
/** Custom indication message sent to the application layer for WMM changes */
#define WMM_CONFIG_CHANGE_INDICATION  "WMM_CONFIG_CHANGE.indication"
		if (!strncmp
		    ((char *)event, WMM_CONFIG_CHANGE_INDICATION,
		     strlen(WMM_CONFIG_CHANGE_INDICATION))) {
			printf("EVENT: STA_DISCONNECTED\n");
			break;
		}
#define CUS_EVT_DRIVER_HANG	            "EVENT=DRIVER_HANG"
		if (!strncmp
		    ((char *)event, CUS_EVT_DRIVER_HANG,
		     strlen(CUS_EVT_DRIVER_HANG))) {
			printf("EVENT: DRIVER_HANG\n");
			break;
		}
#define FW_DEBUG_INFO           "EVENT=FW_DEBUG_INFO"
		if (!strncmp
		    ((char *)event, FW_DEBUG_INFO, strlen(FW_DEBUG_INFO))) {
			printf("EVENT: FW_DEBUG_INFO\n");
			printf("%s\n",
			       (t_u8 *)event + strlen(CUS_EVT_STA_CONNECTED) +
			       1);
			printf("\n");
			break;
		}
		printf("ERR:Undefined event type (0x%X). Dumping event buffer:\n", (unsigned int)event_id);
		hexdump((void *)event, size, ' ');
		break;
	}
	return;
}

/**
 *  @brief Read event data from netlink socket
 *
 *  @param sk_fd    Netlink socket handler
 *  @param buffer   Pointer to the data buffer
 *  @param nlh      Pointer to netlink message header
 *  @param msg      Pointer to message header
 *  @return         Number of bytes read or MLAN_EVENT_FAILURE
 */
int
read_event_netlink_socket(int sk_fd, unsigned char *buffer,
			  struct nlmsghdr *nlh, struct msghdr *msg)
{
	int count = -1;
	count = recvmsg(sk_fd, msg, 0);
#if DEBUG
	mlanevent_printf(MSG_DEBUG, "DBG:Waiting for message from NETLINK.\n");
#endif
	if (count < 0) {
		printf("ERR:NETLINK read failed!\n");
		return MLAN_EVENT_FAILURE;
	}
#if DEBUG
	mlanevent_printf(MSG_DEBUG, "DBG:Received message payload (%d)\n",
			 count);
#endif
	if (count > NLMSG_SPACE(NL_MAX_PAYLOAD)) {
		printf("ERR:Buffer overflow!\n");
		return MLAN_EVENT_FAILURE;
	}
	memset(buffer, 0, NL_MAX_PAYLOAD);
	memcpy(buffer, NLMSG_DATA(nlh), count - NLMSG_HDRLEN);
#if DEBUG
	mlanevent_hexdump(MSG_DEBUG, buffer, count - NLMSG_HDRLEN, ' ');
#endif
	return count - NLMSG_HDRLEN;
}

/**
 *  @brief Configure and read event data from netlink socket
 *
 *  @param sk_fd    Array of netlink sockets
 *  @param no_of_sk Number of netlink sockets opened
 *  @param recv_buf Pointer to the array of evt_buf structures
 *  @param timeout  Socket listen timeout value
 *  @param nlh      Pointer to netlink message header
 *  @param msg      Pointer to message header
 *  @return         Number of bytes read or MLAN_EVENT_FAILURE
 */
int
read_event(int *sk_fd, int no_of_sk, evt_buf *recv_buf, int timeout,
	   struct nlmsghdr *nlh, struct msghdr *msg)
{
	struct timeval tv;
	fd_set rfds;
	int i = 0, max_sk_fd = sk_fd[0];
	int ret = MLAN_EVENT_FAILURE;

	/* Setup read fds */
	FD_ZERO(&rfds);
	for (i = 0; i < no_of_sk; i++) {
		if (sk_fd[i] > max_sk_fd)
			max_sk_fd = sk_fd[i];

		if (sk_fd[i] > 0)
			FD_SET(sk_fd[i], &rfds);
	}

	/* Initialize timeout value */
	if (timeout != 0)
		tv.tv_sec = timeout;
	else
		tv.tv_sec = UAP_RECV_WAIT_DEFAULT;
	tv.tv_usec = 0;

	/* Wait for reply */
	ret = select(max_sk_fd + 1, &rfds, NULL, NULL, &tv);
	if (ret == -1) {
		/* Error */
		terminate_flag++;
		return MLAN_EVENT_FAILURE;
	} else if (!ret) {
		/* Timeout. Try again */
		return MLAN_EVENT_FAILURE;
	}
	for (i = 0; i < no_of_sk; i++) {
		if (sk_fd[i] > 0) {
			if (FD_ISSET(sk_fd[i], &rfds)) {
				/* Success */
				recv_buf[i].flag = 1;
				recv_buf[i].length =
					read_event_netlink_socket(sk_fd[i],
								  recv_buf[i].
								  buffer, nlh,
								  msg);
				ret += recv_buf[i].length;
			}
		}
	}
	return ret;
}

/* Command line options */
static const struct option long_opts[] = {
	{"help", no_argument, NULL, 'h'},
	{"version", no_argument, NULL, 'v'},
	{NULL, 0, NULL, 0}
};

/**
 *  @brief Determine the netlink number
 *
 *  @param i socket number
 *
 *  @return         Netlink number to use
 */
static int
get_netlink_num(int i)
{
	FILE *fp;
	int netlink_num = NETLINK_NXP;
	char str[64];
	char *srch = "netlink_num";
	char filename[64];

	if (i == 0) {
		strcpy(filename, "/proc/mwlan/config");
	} else if (i > 0) {
		sprintf(filename, "/proc/mwlan/config%d", i);
	}
	/* Try to open /proc/mwlan/config$ */
	fp = fopen(filename, "r");
	if (fp) {
		while (fgets(str, sizeof(str), fp)) {
			if (strncmp(str, srch, strlen(srch)) == 0) {
				netlink_num = atoi(str + strlen(srch) + 1);
				break;
			}
		}
		fclose(fp);
	} else {
		return -1;
	}
	printf("Netlink number = %d\n", netlink_num);
	return netlink_num;
}

/****************************************************************************
        Global functions
****************************************************************************/
/**
 *  @brief The main function
 *
 *  @param argc     Number of arguments
 *  @param argv     Pointer to the arguments
 *  @return         0 or 1
 */
int
main(int argc, char *argv[])
{
	int opt;
	int nl_sk[MAX_NO_OF_DEVICES];
	struct nlmsghdr *nlh = NULL;
	struct sockaddr_nl src_addr, dest_addr;
	struct msghdr msg;
	struct iovec iov;
	evt_buf evt_recv_buf[MAX_NO_OF_DEVICES];
	struct timeval current_time;
	struct tm *timeinfo;
	int num_events = 0;
	event_header *event = NULL;
	int ret = MLAN_EVENT_FAILURE;
	int netlink_num[MAX_NO_OF_DEVICES];
	char if_name[IFNAMSIZ + 1];
	t_u32 event_id = 0;
	int i = 0, no_of_sk = 0, dev_index = -1;

	/* Check command line options */
	while ((opt = getopt_long(argc, argv, "hvti:d:", long_opts, NULL)) > 0) {
		switch (opt) {
		case 'h':
			print_usage();
			return 0;
		case 'v':
			printf("mlanevent version : %s\n", MLAN_EVENT_VERSION);
			return 0;
			break;
		case 'i':
			if ((IS_HEX_OR_DIGIT(optarg) == MLAN_EVENT_FAILURE)
			    || ((A2HEXDECIMAL(optarg) >= MAX_NO_OF_DEVICES) &&
				(A2HEXDECIMAL(optarg) != 0xff))) {
				print_usage();
				return 1;
			} else {
				dev_index = A2HEXDECIMAL(optarg);
			}
			break;
		case 'd':
			debug_level = strtoul(optarg, NULL, 10);
#if DEBUG
			mlanevent_printf(MSG_DEBUG, "debug_level=%x\n",
					 debug_level);
#endif
			break;
		default:
			print_usage();
			return 1;
		}
	}

	if (optind < argc) {
		fputs("Too many arguments.\n", stderr);
		print_usage();
		return 1;
	}
	if ((dev_index >= 0) && (dev_index < MAX_NO_OF_DEVICES)) {
		no_of_sk = 1;
	} else {
		/* Currently, we support maximum 4 devices */
		/* TODO: determine no_of_sk at run time */
		no_of_sk = MAX_NO_OF_DEVICES;
	}

	for (i = 0; i < no_of_sk; i++) {
		/* Initialise */
		nl_sk[i] = -1;
		if (no_of_sk == 1) {
			netlink_num[i] = get_netlink_num(dev_index);
			if (netlink_num[i] < 0) {
				printf("ERR:Could not get netlink socket. Invalid device number.\n");
				ret = MLAN_EVENT_FAILURE;
				goto done;
			}
		} else {
			netlink_num[i] = get_netlink_num(i);
		}
		if (netlink_num[i] >= 0) {
			/* Open netlink socket */
			nl_sk[i] = socket(PF_NETLINK, SOCK_RAW, netlink_num[i]);
			if (nl_sk[i] < 0) {
				printf("ERR:Could not open netlink socket.\n");
				ret = MLAN_EVENT_FAILURE;
				goto done;
			}

			/* Set source address */
			memset(&src_addr, 0, sizeof(src_addr));
			src_addr.nl_family = AF_NETLINK;
			src_addr.nl_pid = getpid();	/* Our PID */
			src_addr.nl_groups = NL_MULTICAST_GROUP;

			/* Bind socket with source address */
			if (bind
			    (nl_sk[i], (struct sockaddr *)&src_addr,
			     sizeof(src_addr)) < 0) {
				printf("ERR:Could not bind socket!\n");
				ret = MLAN_EVENT_FAILURE;
				goto done;
			}

			/* Set destination address */
			memset(&dest_addr, 0, sizeof(dest_addr));
			dest_addr.nl_family = AF_NETLINK;
			dest_addr.nl_pid = 0;	/* Kernel */
			dest_addr.nl_groups = NL_MULTICAST_GROUP;

			/* Initialize netlink header */
			nlh = (struct nlmsghdr *)
				malloc(NLMSG_SPACE(NL_MAX_PAYLOAD));
			if (!nlh) {
				printf("ERR: Could not alloc buffer\n");
				ret = MLAN_EVENT_FAILURE;
				goto done;
			}
			memset(nlh, 0, NLMSG_SPACE(NL_MAX_PAYLOAD));

			/* Initialize I/O vector */
			iov.iov_base = (void *)nlh;
			iov.iov_len = NLMSG_SPACE(NL_MAX_PAYLOAD);

			/* Initialize message header */
			memset(&msg, 0, sizeof(struct msghdr));
			msg.msg_name = (void *)&dest_addr;
			msg.msg_namelen = sizeof(dest_addr);
			msg.msg_iov = &iov;
			msg.msg_iovlen = 1;

			memset(&evt_recv_buf[i], 0, sizeof(evt_buf));
		}
	}
	gettimeofday(&current_time, NULL);

	printf("\n");
	printf("**********************************************\n");
	if ((timeinfo = localtime(&(current_time.tv_sec))))
		printf("mlanevent start time : %s", asctime(timeinfo));
	printf("                      %u usecs\n",
	       (unsigned int)current_time.tv_usec);
	printf("**********************************************\n");

	signal(SIGTERM, sig_handler);
	signal(SIGINT, sig_handler);
	signal(SIGALRM, sig_handler);
	while (1) {
		if (terminate_flag) {
			printf("Stopping!\n");
			break;
		}
		ret = read_event(nl_sk, no_of_sk, evt_recv_buf, 0, nlh, &msg);

		/* No result. Loop again */
		if (ret == MLAN_EVENT_FAILURE) {
			continue;
		}
		if (ret == 0) {
			/* Zero bytes received */
			printf("ERR:Received zero bytes!\n");
			continue;
		}
		for (i = 0; i < no_of_sk; i++) {
			if (evt_recv_buf[i].flag == 1) {
				num_events++;
				gettimeofday(&current_time, NULL);
				printf("\n");
				printf("============================================\n");
				printf("Received event");
				if ((timeinfo =
				     localtime(&(current_time.tv_sec))))
					printf(": %s", asctime(timeinfo));
				printf("                     %u usecs\n",
				       (unsigned int)current_time.tv_usec);
				printf("============================================\n");

				memcpy(&event_id, evt_recv_buf[i].buffer,
				       sizeof(event_id));
				if (((event_id & 0xFF000000) == 0x80000000) ||
				    ((event_id & 0xFF000000) == 0)) {
					event = (event_header
						 *)(evt_recv_buf[i].buffer);
				} else {
					memset(if_name, 0, IFNAMSIZ + 1);
					memcpy(if_name, evt_recv_buf[i].buffer,
					       IFNAMSIZ);
					printf("EVENT for interface %s\n",
					       if_name);
					event = (event_header
						 *)((t_u8 *)(evt_recv_buf[i].
							     buffer) +
						    IFNAMSIZ);
					ret -= IFNAMSIZ;
					evt_recv_buf[i].length -= IFNAMSIZ;
				}
#if DEBUG
				mlanevent_printf(MSG_DEBUG,
						 "DBG:Received buffer =\n");
				mlanevent_hexdump(MSG_DEBUG,
						  evt_recv_buf[i].buffer,
						  evt_recv_buf[i].length +
						  IFNAMSIZ, ' ');
#endif
				print_event(event, evt_recv_buf[i].length);
				/* Reset event flag after reading */
				evt_recv_buf[i].flag = 0;
			}
		}
		fflush(stdout);
	}
	gettimeofday(&current_time, NULL);
	printf("\n");
	printf("*********************************************\n");
	if ((timeinfo = localtime(&(current_time.tv_sec))))
		printf("mlanevent end time  : %s", asctime(timeinfo));
	printf("                     %u usecs\n",
	       (unsigned int)current_time.tv_usec);
	printf("Total events       : %u\n", num_events);
	printf("*********************************************\n");
done:
	for (i = 0; i < no_of_sk; i++) {
		if (nl_sk[i] > 0)
			close(nl_sk[i]);
	}
	if (nlh)
		free(nlh);
	return 0;
}
