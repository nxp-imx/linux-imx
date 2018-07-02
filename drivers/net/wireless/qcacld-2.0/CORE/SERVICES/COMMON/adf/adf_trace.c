/*
 * Copyright (c) 2016-2017 The Linux Foundation. All rights reserved.
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

/**
 *  DOC:  adf_trace
 *
 *  ADF trace APIs
 *
 *  Trace, logging, and debugging definitions and APIs
 *
 */

 /* Include Files */
#include <adf_trace.h>
#include "adf_nbuf.h"
#include "adf_os_time.h"
#include "vos_trace.h"
#include "vos_packet.h"
#include "sirDebug.h"
#include "debug_linux.h"
#include "adf_os_io.h"
#include "vos_timer.h"
#include "osdep.h"


/* Static and Global variables */
static spinlock_t l_dp_trace_lock;

static struct adf_dp_trace_record_s
			g_adf_dp_trace_tbl[MAX_ADF_DP_TRACE_RECORDS];

/*
 * all the options to configure/control DP trace are
 * defined in this structure
 */
static struct s_adf_dp_trace_data g_adf_dp_trace_data;
/*
 * all the call back functions for dumping DPTRACE messages from ring buffer
 * are stored in adf_dp_trace_cb_table, callbacks are initialized during init
 */
static tp_adf_dp_trace_cb adf_dp_trace_cb_table[ADF_DP_TRACE_MAX];

/**
 * adf_dp_trace_init() - enables the DP trace
 * Called during driver load and it enables DP trace
 *
 * Return: None
 */
void adf_dp_trace_init(void)
{
	uint8_t i;

	spin_lock_init(&l_dp_trace_lock);
	g_adf_dp_trace_data.head = INVALID_ADF_DP_TRACE_ADDR;
	g_adf_dp_trace_data.tail = INVALID_ADF_DP_TRACE_ADDR;
	g_adf_dp_trace_data.num = 0;
	g_adf_dp_trace_data.proto_bitmap = NBUF_PKT_TRAC_TYPE_EAPOL |
					   NBUF_PKT_TRAC_TYPE_DHCP |
					   NBUF_PKT_TRAC_TYPE_MGMT_ACTION |
					   NBUF_PKT_TRAC_TYPE_ARP;
	g_adf_dp_trace_data.no_of_record = 0;
	g_adf_dp_trace_data.verbosity    = ADF_DP_TRACE_VERBOSITY_HIGH;
	g_adf_dp_trace_data.enable = true;
	g_adf_dp_trace_data.tx_count = 0;
	g_adf_dp_trace_data.rx_count = 0;
	g_adf_dp_trace_data.live_mode = 0;

	for (i = 0; i < ADF_DP_TRACE_MAX; i++)
		adf_dp_trace_cb_table[i] = adf_dp_display_record;

	adf_dp_trace_cb_table[ADF_DP_TRACE_TXRX_PACKET_PTR_RECORD] =
	adf_dp_trace_cb_table[ADF_DP_TRACE_FREE_PACKET_PTR_RECORD] =
				adf_dp_display_ptr_record;
	adf_dp_trace_cb_table[ADF_DP_TRACE_EAPOL_PACKET_RECORD] =
	adf_dp_trace_cb_table[ADF_DP_TRACE_DHCP_PACKET_RECORD] =
	adf_dp_trace_cb_table[ADF_DP_TRACE_ARP_PACKET_RECORD] =
				adf_dp_display_proto_pkt;
	adf_dp_trace_cb_table[ADF_DP_TRACE_MGMT_PACKET_RECORD] =
				adf_dp_display_mgmt_pkt;
	adf_dp_trace_cb_table[ADF_DP_TRACE_EVENT_RECORD] =
				adf_dp_display_event_record;
}

/**
 * adf_dp_trace_set_value() - Configure the value to control DP trace
 * @proto_bitmap  : defines the protocol to be tracked
 * @no_of_records : defines the nth packet which is traced
 * @verbosity     : defines the verbosity level
 *
 * Return: None
 */
void adf_dp_trace_set_value(uint8_t proto_bitmap, uint8_t no_of_record,
			 uint8_t verbosity)
{
	SPIN_LOCK_BH(&l_dp_trace_lock);
	g_adf_dp_trace_data.proto_bitmap = proto_bitmap;
	g_adf_dp_trace_data.no_of_record = no_of_record;
	g_adf_dp_trace_data.verbosity    = verbosity;
	SPIN_UNLOCK_BH(&l_dp_trace_lock);
}

/**
 * adf_dp_trace_enable_track() - enable the tracing for netbuf
 * @code : defines the event
 *
 * Return: true or false depends on whether tracing enabled
 */
static bool adf_dp_trace_enable_track(enum ADF_DP_TRACE_ID code)
{
	switch (g_adf_dp_trace_data.verbosity) {
	case ADF_DP_TRACE_VERBOSITY_HIGH:
		return true;
	case ADF_DP_TRACE_VERBOSITY_MEDIUM:
		if (code <= ADF_DP_TRACE_MED_VERBOSITY)
			return true;
		return false;
	case ADF_DP_TRACE_VERBOSITY_LOW:
		if (code <= ADF_DP_TRACE_LOW_VERBOSITY)
			return true;
		return false;
	case ADF_DP_TRACE_VERBOSITY_DEFAULT:
		if (code <= ADF_DP_TRACE_DEFAULT_VERBOSITY)
			return true;
		return false;
	default:
		return false;
	}
}

/**
 * qdf_dp_get_proto_bitmap() - get dp trace proto bitmap
 *
 * Return: proto bitmap
 */
uint8_t adf_dp_get_proto_bitmap(void)
{
	if (g_adf_dp_trace_data.enable)
		return g_adf_dp_trace_data.proto_bitmap;
	else
		return 0;
}

/**
 * adf_dp_trace_set_track() - Marks whether the packet needs to be traced
 * @nbuf  : defines the netbuf
 * @dir: direction
 *
 * Return: None
 */
void adf_dp_trace_set_track(adf_nbuf_t nbuf,  enum adf_proto_dir dir)
{
	uint32_t count = 0;

	SPIN_LOCK_BH(&l_dp_trace_lock);
	if (ADF_TX == dir)
		count = ++g_adf_dp_trace_data.tx_count;
	else if (ADF_RX == dir)
		count = ++g_adf_dp_trace_data.rx_count;

	if ((g_adf_dp_trace_data.no_of_record != 0) &&
		(count % g_adf_dp_trace_data.no_of_record == 0)) {
		if (ADF_TX == dir)
			ADF_NBUF_CB_TX_DP_TRACE(nbuf) = 1;
		else if (ADF_RX == dir)
			ADF_NBUF_CB_RX_DP_TRACE(nbuf) = 1;
	}
	SPIN_UNLOCK_BH(&l_dp_trace_lock);
}

#define DPTRACE_PRINT(args...) \
	VOS_TRACE(VOS_MODULE_ID_ADF, VOS_TRACE_LEVEL_INFO, ## args)

/**
 * dump_hex_trace() - Display the data in buffer
 * @str:     string to print
 * @buf:     buffer which contains data to be displayed
 * @buf_len: defines the size of the data to be displayed
 *
 * Return: None
 */
static void dump_hex_trace(char *str, uint8_t *buf, uint8_t buf_len)
{
	unsigned char linebuf[BUFFER_SIZE];
	const u8 *ptr = buf;
	int i, linelen, remaining = buf_len;

	/* Dump the bytes in the last line */
	for (i = 0; i < buf_len; i += ROW_SIZE) {
		linelen = min(remaining, ROW_SIZE);
		remaining -= ROW_SIZE;

		hex_dump_to_buffer(ptr + i, linelen, ROW_SIZE, 1,
				linebuf, sizeof(linebuf), false);

		DPTRACE_PRINT("DPT: %s: %s", str, linebuf);
	}
}

/**
 * adf_dp_code_to_string() - convert dptrace code to string
 * @code: dptrace code
 *
 * Return: string version of code
 */
const char *adf_dp_code_to_string(enum ADF_DP_TRACE_ID code)
{
	switch (code) {
	case ADF_DP_TRACE_DROP_PACKET_RECORD:
		return "DROP:";
	case ADF_DP_TRACE_EAPOL_PACKET_RECORD:
		return "EAPOL:";
	case ADF_DP_TRACE_DHCP_PACKET_RECORD:
		return "DHCP:";
	case ADF_DP_TRACE_ARP_PACKET_RECORD:
		return "ARP:";
	case ADF_DP_TRACE_MGMT_PACKET_RECORD:
		return "MGMT:";
	case ADF_DP_TRACE_EVENT_RECORD:
		return "EVENT:";
	case ADF_DP_TRACE_HDD_TX_PACKET_PTR_RECORD:
		return "HDD: TX: PTR:";
	case ADF_DP_TRACE_HDD_TX_PACKET_RECORD:
		return "HDD: TX: DATA:";
	case ADF_DP_TRACE_CE_PACKET_PTR_RECORD:
		return "CE: TX: PTR:";
	case ADF_DP_TRACE_FREE_PACKET_PTR_RECORD:
		return "FREE: TX: PTR:";
	case ADF_DP_TRACE_RX_HTT_PACKET_PTR_RECORD:
		return "HTT: RX: PTR:";
	case ADF_DP_TRACE_RX_OFFLOAD_HTT_PACKET_PTR_RECORD:
		return "HTT: RX: OF: PTR:";
	case ADF_DP_TRACE_RX_HDD_PACKET_PTR_RECORD:
		return "HDD: RX: PTR:";
	case ADF_DP_TRACE_HDD_RX_PACKET_RECORD:
		return "HDD: RX: DATA:";
	case ADF_DP_TRACE_TXRX_QUEUE_PACKET_PTR_RECORD:
		return "TXRX: TX: Q: PTR:";
	case ADF_DP_TRACE_TXRX_PACKET_PTR_RECORD:
		return "TXRX: TX: PTR:";
	case ADF_DP_TRACE_HTT_PACKET_PTR_RECORD:
		return "HTT: TX: PTR:";
	case ADF_DP_TRACE_HTC_PACKET_PTR_RECORD:
		return "HTC: TX: PTR:";
	case ADF_DP_TRACE_HIF_PACKET_PTR_RECORD:
		return "HIF: TX: PTR:";
	case ADF_DP_TRACE_RX_TXRX_PACKET_PTR_RECORD:
		return "TXRX: RX: PTR:";
	case ADF_DP_TRACE_HDD_TX_TIMEOUT:
		return "HDD: STA: TO:";
	case ADF_DP_TRACE_HDD_SOFTAP_TX_TIMEOUT:
		return "HDD: SAP: TO:";
	default:
		return "Invalid";
	}
}

/**
 * adf_dp_dir_to_str() - convert direction to string
 * @dir: direction
 *
 * Return: string version of direction
 */
const char *adf_dp_dir_to_str(enum adf_proto_dir dir)
{
	switch (dir) {
	case ADF_TX:
		return " --> ";
	case ADF_RX:
		return " <-- ";
	default:
		return "invalid";
	}
}

/**
 * adf_dp_type_to_str() - convert packet type to string
 * @type: type
 *
 * Return: string version of packet type
 */
const char *adf_dp_type_to_str(enum adf_proto_type type)
{
	switch (type) {
	case ADF_PROTO_TYPE_DHCP:
		return "DHCP";
	case ADF_PROTO_TYPE_EAPOL:
		return "EAPOL";
	case ADF_PROTO_TYPE_ARP:
		return "ARP";
	case ADF_PROTO_TYPE_MGMT:
		return "MGMT";
	case ADF_PROTO_TYPE_EVENT:
		return "EVENT";
	default:
		return "invalid";
	}
}

/**
 * adf_dp_subtype_to_str() - convert packet subtype to string
 * @type: type
 *
 * Return: string version of packet subtype
 */
const char *adf_dp_subtype_to_str(enum adf_proto_subtype subtype)
{
	switch (subtype) {
	case ADF_PROTO_EAPOL_M1:
		return "M1";
	case ADF_PROTO_EAPOL_M2:
		return "M2";
	case ADF_PROTO_EAPOL_M3:
		return "M3";
	case ADF_PROTO_EAPOL_M4:
		return "M4";
	case ADF_PROTO_DHCP_DISCOVER:
		return "DISCOVER";
	case ADF_PROTO_DHCP_REQUEST:
		return "REQUEST";
	case ADF_PROTO_DHCP_OFFER:
		return "OFFER";
	case ADF_PROTO_DHCP_ACK:
		return "ACK";
	case ADF_PROTO_DHCP_NACK:
		return "NACK";
	case ADF_PROTO_DHCP_RELEASE:
		return "RELEASE";
	case ADF_PROTO_DHCP_INFORM:
		return "INFORM";
	case ADF_PROTO_DHCP_DECLINE:
		return "DECLINE";
	case ADF_PROTO_ARP_REQ:
		return "REQUEST";
	case ADF_PROTO_ARP_RES:
		return "RESPONSE";
	case ADF_PROTO_MGMT_ASSOC:
		return "ASSOC";
	case ADF_PROTO_MGMT_DISASSOC:
		return "DISASSOC";
	case ADF_PROTO_MGMT_AUTH:
		return "AUTH";
	case ADF_PROTO_MGMT_DEAUTH:
		return "DEAUTH";
	case ADF_ROAM_SYNCH:
		return "ROAM SYNCH";
	case ADF_ROAM_COMPLETE:
		return "ROAM COMPLETE";
	case ADF_ROAM_EVENTID:
		return "ROAM EVENTID";
	default:
		return "invalid";
	}
}

/**
 * adf_dp_enable_check() - check if dptrace is enable or not
 * @nbuf: nbuf
 * @code: dptrace code
 *
 * Return: true/false
 */
bool adf_dp_enable_check(adf_nbuf_t nbuf, enum ADF_DP_TRACE_ID code,
		enum adf_proto_dir dir)
{
	/* Return when Dp trace is not enabled */
	if (!g_adf_dp_trace_data.enable)
		return false;

	if (adf_dp_trace_enable_track(code) == false)
		return false;

	if ((nbuf) && ((NBUF_GET_PACKET_TRACK(nbuf) !=
		 NBUF_TX_PKT_DATA_TRACK) ||
		 ((dir == ADF_TX) && (ADF_NBUF_CB_TX_DP_TRACE(nbuf) == 0)) ||
		 ((dir == ADF_RX) && (ADF_NBUF_CB_RX_DP_TRACE(nbuf) == 0))))
		return false;

	return true;
}

/**
 * adf_dp_add_record() - add dp trace record
 * @code: dptrace code
 * @data: data pointer
 * @size: size of buffer
 * @print: print it in kmsg
 *
 * Return: none
 */
void adf_dp_add_record(enum ADF_DP_TRACE_ID code,
		       uint8_t *data, uint8_t size, bool print)
{
	struct adf_dp_trace_record_s *rec = NULL;
	int index;

	SPIN_LOCK_BH(&l_dp_trace_lock);

	g_adf_dp_trace_data.num++;

	if (g_adf_dp_trace_data.num > MAX_ADF_DP_TRACE_RECORDS)
		g_adf_dp_trace_data.num = MAX_ADF_DP_TRACE_RECORDS;

	if (INVALID_ADF_DP_TRACE_ADDR == g_adf_dp_trace_data.head) {
		/* first record */
		g_adf_dp_trace_data.head = 0;
		g_adf_dp_trace_data.tail = 0;
	} else {
		/* queue is not empty */
		g_adf_dp_trace_data.tail++;

		if (MAX_ADF_DP_TRACE_RECORDS == g_adf_dp_trace_data.tail)
			g_adf_dp_trace_data.tail = 0;

		if (g_adf_dp_trace_data.head == g_adf_dp_trace_data.tail) {
			/* full */
			if (MAX_ADF_DP_TRACE_RECORDS ==
				++g_adf_dp_trace_data.head)
				g_adf_dp_trace_data.head = 0;
		}
	}

	rec = &g_adf_dp_trace_tbl[g_adf_dp_trace_data.tail];
	index = g_adf_dp_trace_data.tail;
	rec->code = code;
	rec->size = 0;
	if (data != NULL && size > 0) {
		if (size > ADF_DP_TRACE_RECORD_SIZE)
			size = ADF_DP_TRACE_RECORD_SIZE;

		rec->size = size;
		adf_os_mem_copy(rec->data, data, size);

	}
	vos_get_time_of_the_day_in_hr_min_sec_usec(rec->time,
					sizeof(rec->time));
	rec->pid = (in_interrupt() ? 0 : current->pid);
	SPIN_UNLOCK_BH(&l_dp_trace_lock);

	if ((g_adf_dp_trace_data.live_mode || print == true) &&
	    (rec->code < ADF_DP_TRACE_MAX))
		adf_dp_trace_cb_table[rec->code] (rec, index);
}

/**
 * adf_log_eapol_pkt() - log EAPOL packet
 * @session_id: vdev_id
 * @skb: skb pointer
 * @dir: direction
 *
 * Return: true/false
 */
bool adf_log_eapol_pkt(uint8_t session_id, struct sk_buff *skb,
		       enum adf_proto_dir dir)
{
	enum adf_proto_subtype subtype;

	if ((adf_dp_get_proto_bitmap() & NBUF_PKT_TRAC_TYPE_EAPOL) &&
		((dir == ADF_TX && ADF_NBUF_GET_IS_EAPOL(skb)) ||
		(dir == ADF_RX && adf_nbuf_is_eapol_pkt(skb)))) {

		subtype = adf_nbuf_get_eapol_subtype(skb);
		DPTRACE(adf_dp_trace_proto_pkt(ADF_DP_TRACE_EAPOL_PACKET_RECORD,
			session_id, (skb->data + ADF_NBUF_SRC_MAC_OFFSET),
			(skb->data + ADF_NBUF_DEST_MAC_OFFSET),
			ADF_PROTO_TYPE_EAPOL, subtype,
			dir));
		if (ADF_TX == dir)
			ADF_NBUF_CB_TX_DP_TRACE(skb) = 1;
		else if (ADF_RX == dir)
			ADF_NBUF_CB_RX_DP_TRACE(skb) = 1;

		ADF_NBUF_CB_DP_TRACE_PRINT(skb) = true;
		return true;
	}
	return false;
}

/**
 * adf_log_dhcp_pkt() - log DHCP packet
 * @session_id: vdev_id
 * @skb: skb pointer
 * @dir: direction
 *
 * Return: true/false
 */
bool adf_log_dhcp_pkt(uint8_t session_id, struct sk_buff *skb,
		      enum adf_proto_dir dir)
{
	enum adf_proto_subtype subtype = ADF_PROTO_INVALID;

	if ((adf_dp_get_proto_bitmap() & NBUF_PKT_TRAC_TYPE_DHCP) &&
		((dir == ADF_TX && ADF_NBUF_GET_IS_DHCP(skb)) ||
		(dir == ADF_RX && adf_nbuf_is_dhcp_pkt(skb)))) {

		subtype = adf_nbuf_get_dhcp_subtype(skb);
		DPTRACE(adf_dp_trace_proto_pkt(ADF_DP_TRACE_DHCP_PACKET_RECORD,
			session_id, (skb->data + ADF_NBUF_SRC_MAC_OFFSET),
			(skb->data + ADF_NBUF_DEST_MAC_OFFSET),
			ADF_PROTO_TYPE_DHCP, subtype,
			dir));
		if (ADF_TX == dir)
			ADF_NBUF_CB_TX_DP_TRACE(skb) = 1;
		else if (ADF_RX == dir)
			ADF_NBUF_CB_RX_DP_TRACE(skb) = 1;

		ADF_NBUF_CB_DP_TRACE_PRINT(skb) = true;
		return true;
	}
	return false;
}

/**
 * adf_log_arp_pkt() - log ARP packet
 * @session_id: vdev_id
 * @skb: skb pointer
 * @dir: direction
 *
 * Return: true/false
 */
bool adf_log_arp_pkt(uint8_t session_id, struct sk_buff *skb,
		     enum adf_proto_dir dir)
{
	enum adf_proto_subtype proto_subtype;

	if ((adf_dp_get_proto_bitmap() & NBUF_PKT_TRAC_TYPE_ARP) &&
	    ((dir == ADF_TX && ADF_NBUF_GET_IS_ARP(skb)) ||
	    (dir == ADF_RX && adf_nbuf_is_ipv4_arp_pkt(skb)))){

		proto_subtype = adf_nbuf_get_arp_subtype(skb);

		DPTRACE(adf_dp_trace_proto_pkt(ADF_DP_TRACE_ARP_PACKET_RECORD,
			session_id, (skb->data + ADF_NBUF_SRC_MAC_OFFSET),
			(skb->data + ADF_NBUF_DEST_MAC_OFFSET),
			ADF_PROTO_TYPE_ARP, proto_subtype,
			dir));
		if (ADF_TX == dir)
			ADF_NBUF_CB_TX_DP_TRACE(skb) = 1;
		else if (ADF_RX == dir)
			ADF_NBUF_CB_RX_DP_TRACE(skb) = 1;

		ADF_NBUF_CB_DP_TRACE_PRINT(skb) = true;
		return true;
	}
	return false;
}

/**
 * adf_dp_trace_log_pkt() - log packet type enabled through iwpriv
 * @session_id: vdev_id
 * @skb: skb pointer
 * @dir: direction
 *
 * Return: none
 */
void adf_dp_trace_log_pkt(uint8_t session_id, struct sk_buff *skb,
			  enum adf_proto_dir dir)
{
	if (adf_dp_get_proto_bitmap()) {
		if (adf_log_arp_pkt(session_id,
			skb, dir) == false) {
			if (adf_log_dhcp_pkt(session_id,
				skb, dir) == false) {
				if (adf_log_eapol_pkt(session_id,
					skb, dir) == false) {
					return;
				}
			}
		}
	}
}

/**
 * adf_dp_display_proto_pkt() - display proto packet
 * @record: dptrace record
 * @index: index
 *
 * Return: none
 */
void adf_dp_display_proto_pkt(struct adf_dp_trace_record_s *record,
			      uint16_t index)
{
	struct adf_dp_trace_proto_buf *buf =
		(struct adf_dp_trace_proto_buf *)record->data;

	DPTRACE_PRINT("DPT: %04d: %s: %s vdev_id %d\n", index,
		record->time, adf_dp_code_to_string(record->code),
		buf->vdev_id);
	DPTRACE_PRINT("DPT: SA: " MAC_ADDRESS_STR " %s DA: " MAC_ADDRESS_STR
						" Type %s Subtype %s\n",
		MAC_ADDR_ARRAY(buf->sa.bytes), adf_dp_dir_to_str(buf->dir),
		MAC_ADDR_ARRAY(buf->da.bytes), adf_dp_type_to_str(buf->type),
		adf_dp_subtype_to_str(buf->subtype));
}

/**
 * adf_dp_trace_proto_pkt() - record proto packet
 * @code: dptrace code
 * @vdev_id: vdev id
 * @sa: source mac address
 * @da: destination mac address
 * @type: proto type
 * @subtype: proto subtype
 * @dir: direction
 *
 * Return: none
 */
void adf_dp_trace_proto_pkt(enum ADF_DP_TRACE_ID code, uint8_t vdev_id,
		uint8_t *sa, uint8_t *da, enum adf_proto_type type,
		enum adf_proto_subtype subtype, enum adf_proto_dir dir)
{
	struct adf_dp_trace_proto_buf buf;
	int buf_size = sizeof(struct adf_dp_trace_ptr_buf);

	if (adf_dp_enable_check(NULL, code, dir) == false)
		return;

	if (buf_size > ADF_DP_TRACE_RECORD_SIZE)
		ADF_BUG(0);

	memcpy(&buf.sa, sa, ETH_ALEN);
	memcpy(&buf.da, da, ETH_ALEN);
	buf.dir = dir;
	buf.type = type;
	buf.subtype = subtype;
	buf.vdev_id = vdev_id;
	adf_dp_add_record(code, (uint8_t *)&buf, buf_size, true);
}

void adf_dp_display_mgmt_pkt(struct adf_dp_trace_record_s *record,
			      uint16_t index)
{
	struct adf_dp_trace_mgmt_buf *buf =
		(struct adf_dp_trace_mgmt_buf *)record->data;

	DPTRACE_PRINT("DPT: %04d: %s: %s vdev_id %d", index,
		record->time, adf_dp_code_to_string(record->code),
		buf->vdev_id);
	DPTRACE_PRINT("DPT: Type %s Subtype %s", adf_dp_type_to_str(buf->type),
		adf_dp_subtype_to_str(buf->subtype));
}

void adf_dp_trace_mgmt_pkt(enum ADF_DP_TRACE_ID code, uint8_t vdev_id,
		enum adf_proto_type type, enum adf_proto_subtype subtype)
{
	struct adf_dp_trace_mgmt_buf buf;
	int buf_size = sizeof(struct adf_dp_trace_mgmt_buf);

	if (adf_dp_enable_check(NULL, code, ADF_NA) == false)
		return;

	if (buf_size > ADF_DP_TRACE_RECORD_SIZE)
		ADF_BUG(0);

	buf.type = type;
	buf.subtype = subtype;
	buf.vdev_id = vdev_id;
	adf_dp_add_record(code, (uint8_t *)&buf, buf_size, true);
}

void adf_dp_display_event_record(struct adf_dp_trace_record_s *record,
			      uint16_t index)
{
	struct adf_dp_trace_event_buf *buf =
		(struct adf_dp_trace_event_buf *)record->data;

	DPTRACE_PRINT("DPT: %04d: %s: %s vdev_id %d", index,
		record->time, adf_dp_code_to_string(record->code),
		buf->vdev_id);
	DPTRACE_PRINT("DPT: Type %s Subtype %s", adf_dp_type_to_str(buf->type),
		adf_dp_subtype_to_str(buf->subtype));
}

void adf_dp_trace_record_event(enum ADF_DP_TRACE_ID code, uint8_t vdev_id,
		enum adf_proto_type type, enum adf_proto_subtype subtype)
{
	struct adf_dp_trace_event_buf buf;
	int buf_size = sizeof(struct adf_dp_trace_event_buf);

	if (adf_dp_enable_check(NULL, code, ADF_NA) == false)
		return;

	if (buf_size > ADF_DP_TRACE_RECORD_SIZE)
		ADF_BUG(0);

	buf.type = type;
	buf.subtype = subtype;
	buf.vdev_id = vdev_id;
	adf_dp_add_record(code, (uint8_t *)&buf, buf_size, true);
}

/**
 * adf_dp_display_ptr_record() - display record
 * @record: dptrace record
 * @index: index
 *
 * Return: none
 */
void adf_dp_display_ptr_record(struct adf_dp_trace_record_s *record,
				uint16_t index)
{
	struct adf_dp_trace_ptr_buf *buf =
		(struct adf_dp_trace_ptr_buf *)record->data;

	if (record->code == ADF_DP_TRACE_FREE_PACKET_PTR_RECORD)
		DPTRACE_PRINT("DPT: %04d: %s: %s msdu_id: %d, status: %d\n", index,
			record->time, adf_dp_code_to_string(record->code),
			buf->msdu_id, buf->status);
	else
		DPTRACE_PRINT("DPT: %04d: %s: %s msdu_id: %d, vdev_id: %d\n", index,
			record->time, adf_dp_code_to_string(record->code),
			buf->msdu_id, buf->status);

	dump_hex_trace("cookie", (uint8_t *)&buf->cookie, sizeof(buf->cookie));
}

/**
 * adf_dp_trace_ptr() - record dptrace
 * @code: dptrace code
 * @data: data
 * @size: size of data
 * @msdu_id: msdu_id
 * @status: return status
 *
 * Return: none
 */
void adf_dp_trace_ptr(adf_nbuf_t nbuf, enum ADF_DP_TRACE_ID code,
		uint8_t *data, uint8_t size, uint16_t msdu_id, uint16_t status)
{
	struct adf_dp_trace_ptr_buf buf;
	int buf_size = sizeof(struct adf_dp_trace_ptr_buf);

	if (adf_dp_enable_check(nbuf, code, ADF_TX) == false)
		return;

	if (buf_size > ADF_DP_TRACE_RECORD_SIZE)
		ADF_BUG(0);

	adf_os_mem_copy(&buf.cookie, data, size);
	buf.msdu_id = msdu_id;
	buf.status = status;
	adf_dp_add_record(code, (uint8_t *)&buf, buf_size,
				ADF_NBUF_CB_DP_TRACE_PRINT(nbuf));
}

/**
 * adf_dp_display_trace() - Displays a record in DP trace
 * @pRecord  : pointer to a record in DP trace
 * @recIndex : record index
 *
 * Return: None
 */
void adf_dp_display_record(struct adf_dp_trace_record_s *pRecord,
				uint16_t recIndex)
{
	uint8_t rsize = pRecord->size;
	if (rsize > ADF_DP_TRACE_RECORD_SIZE)
		rsize = ADF_DP_TRACE_RECORD_SIZE;

	DPTRACE_PRINT("DPT: %04d: %s: %s\n", recIndex,
		pRecord->time, adf_dp_code_to_string(pRecord->code));
	switch (pRecord->code) {
	case  ADF_DP_TRACE_HDD_TX_TIMEOUT:
		DPTRACE_PRINT("DPT: HDD TX Timeout\n");
		break;
	case  ADF_DP_TRACE_HDD_SOFTAP_TX_TIMEOUT:
		DPTRACE_PRINT("DPT: HDD SoftAP TX Timeout\n");
		break;
	case ADF_DP_TRACE_HDD_TX_PACKET_RECORD:
	case ADF_DP_TRACE_HDD_RX_PACKET_RECORD:
		dump_hex_trace("DATA", pRecord->data, rsize);
		break;
	default:
		dump_hex_trace("cookie", pRecord->data, rsize);
	}
}

/**
 * adf_dp_trace() - Stores the data in buffer
 * @nbuf  : defines the netbuf
 * @code : defines the event
 * @data : defines the data to be stored
 * @size : defines the size of the data record
 *
 * Return: None
 */
void adf_dp_trace(adf_nbuf_t nbuf, enum ADF_DP_TRACE_ID code,
			uint8_t *data, uint8_t size, enum adf_proto_dir dir)
{

	if (adf_dp_enable_check(nbuf, code, dir) == false)
		return;

	adf_dp_add_record(code, data, size,
			nbuf ? ADF_NBUF_CB_DP_TRACE_PRINT(nbuf) : false);
}

/**
 * adf_dp_trace_enable_live_mode() - enable live mode
 *
 * Return: none
 */
void adf_dp_trace_enable_live_mode(void)
{
	g_adf_dp_trace_data.live_mode = 1;
}

void adf_dp_trace_clear_buffer(void)
{
	g_adf_dp_trace_data.head = INVALID_ADF_DP_TRACE_ADDR;
	g_adf_dp_trace_data.tail = INVALID_ADF_DP_TRACE_ADDR;
	g_adf_dp_trace_data.num = 0;
	g_adf_dp_trace_data.proto_bitmap = NBUF_PKT_TRAC_TYPE_EAPOL |
					   NBUF_PKT_TRAC_TYPE_DHCP |
					   NBUF_PKT_TRAC_TYPE_MGMT_ACTION |
					   NBUF_PKT_TRAC_TYPE_ARP;
	g_adf_dp_trace_data.no_of_record = 0;
	g_adf_dp_trace_data.verbosity	 = ADF_DP_TRACE_VERBOSITY_HIGH;
	g_adf_dp_trace_data.enable = true;
	g_adf_dp_trace_data.tx_count = 0;
	g_adf_dp_trace_data.rx_count = 0;
	g_adf_dp_trace_data.live_mode = 0;

	memset(g_adf_dp_trace_tbl, 0,
		MAX_ADF_DP_TRACE_RECORDS * sizeof(struct adf_dp_trace_record_s));

}

/**
 * adf_dp_trace_dump_all() - Dump data from ring buffer via call back functions
 *			  registered with ADF
 * @code : Reason code
 * @count : Number of lines to dump starting from tail to head
 *
 * Return : nothing
 */
void adf_dp_trace_dump_all(uint32_t count)
{
	struct adf_dp_trace_record_s pRecord;
	int32_t i, tail;

	if (!g_adf_dp_trace_data.enable) {
		VOS_TRACE(VOS_MODULE_ID_SYS,
			  VOS_TRACE_LEVEL_ERROR, "Tracing Disabled");
		return;
	}

	VOS_TRACE(VOS_MODULE_ID_SYS, VOS_TRACE_LEVEL_ERROR,
		  "Total Records: %d, Head: %d, Tail: %d",
		  g_adf_dp_trace_data.num, g_adf_dp_trace_data.head,
		  g_adf_dp_trace_data.tail);

	/* aquire the lock so that only one thread at a time can read
	 * the ring buffer
	 */
	SPIN_LOCK_BH(&l_dp_trace_lock);

	if (g_adf_dp_trace_data.head != INVALID_ADF_DP_TRACE_ADDR) {
		i = g_adf_dp_trace_data.head;
		tail = g_adf_dp_trace_data.tail;

		if (count) {
			if (count > g_adf_dp_trace_data.num)
				count = g_adf_dp_trace_data.num;
			if (tail >= (count - 1))
				i = tail - count + 1;
			else if (count != MAX_ADF_DP_TRACE_RECORDS)
				i = MAX_ADF_DP_TRACE_RECORDS - ((count - 1) -
							     tail);
		}

		pRecord = g_adf_dp_trace_tbl[i];
		SPIN_UNLOCK_BH(&l_dp_trace_lock);
		for (;; ) {
			adf_dp_trace_cb_table[pRecord.
					   code] (&pRecord, (uint16_t)i);
			if (i == tail)
				break;
			i += 1;

			SPIN_LOCK_BH(&l_dp_trace_lock);
			if (MAX_ADF_DP_TRACE_RECORDS == i)
				i = 0;

			pRecord = g_adf_dp_trace_tbl[i];
			SPIN_UNLOCK_BH(&l_dp_trace_lock);
		}
	} else {
		SPIN_UNLOCK_BH(&l_dp_trace_lock);
	}
}
