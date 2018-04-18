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

#if !defined(__ADF_TRACE_H)
#define __ADF_TRACE_H

/**
 *  DOC:  adf_trace
 *
 *  Atheros driver framework trace APIs
 *
 *  Trace, logging, and debugging definitions and APIs
 *
 */

 /* Include Files */
#include  <adf_nbuf.h>
#include "vos_types.h"

#ifdef FEATURE_DPTRACE_ENABLE
 /* DP Trace Implementation */
#define DPTRACE(p) p
#else
#define DPTRACE(p)  /*no-op*/
#endif

#define MAX_ADF_DP_TRACE_RECORDS       4000
#define ADF_DP_TRACE_RECORD_SIZE       16
#define INVALID_ADF_DP_TRACE_ADDR      0xffffffff
#define ADF_DP_TRACE_VERBOSITY_HIGH    3
#define ADF_DP_TRACE_VERBOSITY_MEDIUM  2
#define ADF_DP_TRACE_VERBOSITY_LOW     1
#define ADF_DP_TRACE_VERBOSITY_DEFAULT 0

#define DUMP_DP_TRACE       0
#define ENABLE_DP_TRACE_LIVE_MODE	1
#define CLEAR_DP_TRACE_BUFFER	2


/**
 * struct adf_mac_addr - mac address array
 * @bytes: MAC address bytes
 */
struct adf_mac_addr {
	uint8_t bytes[VOS_MAC_ADDR_SIZE];
};

/**
 * enum ADF_DP_TRACE_ID - Generic ID to identify various events in data path
 * @ADF_DP_TRACE_INVALID - invalid
 * @ADF_DP_TRACE_DROP_PACKET_RECORD - record drop packet
 * @ADF_DP_TRACE_EAPOL_PACKET_RECORD - record EAPOL packet
 * @ADF_DP_TRACE_DHCP_PACKET_RECORD - record DHCP packet
 * @ADF_DP_TRACE_ARP_PACKET_RECORD - record ARP packet
 * @ADF_DP_TRACE_MGMT_PACKET_RECORD - record MGMT pacekt
 * @ADF_DP_TRACE_EVENT_RECORD - record events
 * @ADF_DP_TRACE_DEFAULT_VERBOSITY - below this are part of default verbosity
 * @ADF_DP_TRACE_HDD_TX_TIMEOUT - HDD tx timeout
 * @ADF_DP_TRACE_HDD_SOFTAP_TX_TIMEOUT- SOFTAP HDD tx timeout
 * @ADF_DP_TRACE_HDD_TX_PACKET_PTR_RECORD - HDD layer ptr record
 * @ADF_DP_TRACE_CE_PACKET_PTR_RECORD - CE layer ptr record
 * @ADF_DP_TRACE_FREE_PACKET_PTR_RECORD - tx completion ptr record
 * @ADF_DP_TRACE_RX_HTT_PACKET_PTR_RECORD - HTT RX record
 * @ADF_DP_TRACE_RX_OFFLOAD_HTT_PACKET_PTR_RECORD- HTT RX offload record
 * @ADF_DP_TRACE_RX_HDD_PACKET_PTR_RECORD - HDD RX record
 * @ADF_DP_TRACE_LOW_VERBOSITY - below this are part of low verbosity
 * @ADF_DP_TRACE_TXRX_QUEUE_PACKET_PTR_RECORD -tx queue ptr record
 * @ADF_DP_TRACE_TXRX_PACKET_PTR_RECORD - txrx packet ptr record
 * @ADF_DP_TRACE_HTT_PACKET_PTR_RECORD - htt packet ptr record
 * @ADF_DP_TRACE_HTC_PACKET_PTR_RECORD - htc packet ptr record
 * @ADF_DP_TRACE_HIF_PACKET_PTR_RECORD - hif packet ptr record
 * @ADF_DP_TRACE_RX_TXRX_PACKET_PTR_RECORD - txrx packet ptr record
 * @ADF_DP_TRACE_MED_VERBOSITY - below this are part of med verbosity
 * @ADF_DP_TRACE_HDD_TX_PACKET_RECORD - record 32 bytes at HDD
 * @ADF_DP_TRACE_HDD_RX_PACKET_RECORD - record 32 bytes at HDD
 * @ADF_DP_TRACE_HIGH_VERBOSITY - below this are part of high verbosity
 */
enum  ADF_DP_TRACE_ID {
	ADF_DP_TRACE_INVALID = 0,
	ADF_DP_TRACE_DROP_PACKET_RECORD,
	ADF_DP_TRACE_EAPOL_PACKET_RECORD,
	ADF_DP_TRACE_DHCP_PACKET_RECORD,
	ADF_DP_TRACE_ARP_PACKET_RECORD,
	ADF_DP_TRACE_MGMT_PACKET_RECORD,
	ADF_DP_TRACE_EVENT_RECORD,
	ADF_DP_TRACE_DEFAULT_VERBOSITY,
	ADF_DP_TRACE_HDD_TX_TIMEOUT,
	ADF_DP_TRACE_HDD_SOFTAP_TX_TIMEOUT,
	ADF_DP_TRACE_HDD_TX_PACKET_PTR_RECORD,
	ADF_DP_TRACE_CE_PACKET_PTR_RECORD,
	ADF_DP_TRACE_FREE_PACKET_PTR_RECORD,
	ADF_DP_TRACE_RX_HTT_PACKET_PTR_RECORD,
	ADF_DP_TRACE_RX_OFFLOAD_HTT_PACKET_PTR_RECORD,
	ADF_DP_TRACE_RX_HDD_PACKET_PTR_RECORD,
	ADF_DP_TRACE_LOW_VERBOSITY,
	ADF_DP_TRACE_TXRX_QUEUE_PACKET_PTR_RECORD,
	ADF_DP_TRACE_TXRX_PACKET_PTR_RECORD,
	ADF_DP_TRACE_HTT_PACKET_PTR_RECORD,
	ADF_DP_TRACE_HTC_PACKET_PTR_RECORD,
	ADF_DP_TRACE_HIF_PACKET_PTR_RECORD,
	ADF_DP_TRACE_RX_TXRX_PACKET_PTR_RECORD,
	ADF_DP_TRACE_MED_VERBOSITY,
	ADF_DP_TRACE_HDD_TX_PACKET_RECORD,
	ADF_DP_TRACE_HDD_RX_PACKET_RECORD,
	ADF_DP_TRACE_HIGH_VERBOSITY,
	ADF_DP_TRACE_MAX
};

/**
 * adf_proto_dir - direction
 * @ADF_TX: TX direction
 * @ADF_RX: RX direction
 * @ADF_NA: not applicable
 */
enum adf_proto_dir {
	ADF_TX,
	ADF_RX,
	ADF_NA
};

/**
 * struct adf_dp_trace_ptr_buf - pointer record buffer
 * @cookie: cookie value
 * @msdu_id: msdu_id
 * @status: completion status
 */
struct adf_dp_trace_ptr_buf {
	uint64_t cookie;
	uint16_t msdu_id;
	uint16_t status;
};

/**
 * struct adf_dp_trace_proto_buf - proto packet buffer
 * @sa: source address
 * @da: destination address
 * @vdev_id : vdev id
 * @type: packet type
 * @subtype: packet subtype
 * @dir: direction
 */
struct adf_dp_trace_proto_buf {
	struct adf_mac_addr sa;
	struct adf_mac_addr da;
	uint8_t vdev_id;
	uint8_t type;
	uint8_t subtype;
	uint8_t dir;
};

/**
 * struct adf_dp_trace_mgmt_buf - mgmt packet buffer
 * @vdev_id : vdev id
 * @type: packet type
 * @subtype: packet subtype
 */
struct adf_dp_trace_mgmt_buf {
	uint8_t vdev_id;
	uint8_t type;
	uint8_t subtype;
};

/**
 * struct adf_dp_trace_event_buf - event buffer
 * @vdev_id : vdev id
 * @type: packet type
 * @subtype: packet subtype
 */
struct adf_dp_trace_event_buf {
	uint8_t vdev_id;
	uint8_t type;
	uint8_t subtype;
};

/**
 * struct adf_dp_trace_record_s - Describes a record in DP trace
 * @time: time when it got stored
 * @code: Describes the particular event
 * @data: buffer to store data
 * @size: Length of the valid data stored in this record
 * @pid : process id which stored the data in this record
 */
struct adf_dp_trace_record_s {
	char time[20];
	uint8_t code;
	uint8_t data[ADF_DP_TRACE_RECORD_SIZE];
	uint8_t size;
	uint32_t pid;
};

/**
 * struct adf_dp_trace_data - Parameters to configure/control DP trace
 * @head: Position of first record
 * @tail: Position of last record
 * @num:  Current index
 * @proto_bitmap: defines which protocol to be traced
 * @no_of_record: defines every nth packet to be traced
 * @verbosity : defines verbosity level
 * @enable: enable/disable DP trace
 * @count: current packet number
 */
struct s_adf_dp_trace_data {
	uint32_t head;
	uint32_t tail;
	uint32_t num;

	/* config for controlling the trace */
	uint8_t proto_bitmap;
	uint8_t no_of_record;
	uint8_t verbosity;
	bool enable;
	uint32_t tx_count;
	uint32_t rx_count;
	bool live_mode;
};
/* Function declarations and documenation */

#ifdef FEATURE_DPTRACE_ENABLE
void adf_dp_trace_init(void);
void adf_dp_trace_set_value(uint8_t proto_bitmap, uint8_t no_of_records,
			 uint8_t verbosity);
void adf_dp_trace_set_track(adf_nbuf_t nbuf, enum adf_proto_dir dir);
void adf_dp_trace(adf_nbuf_t nbuf, enum ADF_DP_TRACE_ID code,
			uint8_t *data, uint8_t size, enum adf_proto_dir dir);
void adf_dp_trace_dump_all(uint32_t count);
typedef void (*tp_adf_dp_trace_cb)(struct adf_dp_trace_record_s* , uint16_t);
void adf_dp_display_record(struct adf_dp_trace_record_s *record,
							uint16_t index);
void adf_dp_trace_ptr(adf_nbuf_t nbuf, enum ADF_DP_TRACE_ID code,
		uint8_t *data, uint8_t size, uint16_t msdu_id, uint16_t status);

void adf_dp_display_ptr_record(struct adf_dp_trace_record_s *pRecord,
				uint16_t recIndex);
uint8_t adf_dp_get_proto_bitmap(void);
void
adf_dp_trace_proto_pkt(enum ADF_DP_TRACE_ID code, uint8_t vdev_id,
		uint8_t *sa, uint8_t *da, enum adf_proto_type type,
		enum adf_proto_subtype subtype, enum adf_proto_dir dir);
void adf_dp_display_proto_pkt(struct adf_dp_trace_record_s *record,
				uint16_t index);
void adf_dp_trace_log_pkt(uint8_t session_id, struct sk_buff *skb,
				enum adf_proto_dir dir);
void adf_dp_trace_enable_live_mode(void);
void adf_dp_trace_clear_buffer(void);

/**
 * adf_dp_trace_mgmt_pkt() - record mgmt packet
 * @code: dptrace code
 * @vdev_id: vdev id
 * @type: proto type
 * @subtype: proto subtype
 *
 * Return: none
 */
void adf_dp_trace_mgmt_pkt(enum ADF_DP_TRACE_ID code, uint8_t vdev_id,
		enum adf_proto_type type, enum adf_proto_subtype subtype);

/**
 * adf_dp_display_mgmt_pkt() - display proto packet
 * @record: dptrace record
 * @index: index
 *
 * Return: none
 */
void adf_dp_display_mgmt_pkt(struct adf_dp_trace_record_s *record,
			      uint16_t index);

/**
 * adf_dp_display_event_record() - display event records
 * @record: dptrace record
 * @index: index
 *
 * Return: none
 */
void adf_dp_display_event_record(struct adf_dp_trace_record_s *record,
			      uint16_t index);

/**
 * adf_dp_trace_record_event() - record events
 * @code: dptrace code
 * @vdev_id: vdev id
 * @type: proto type
 * @subtype: proto subtype
 *
 * Return: none
 */
void adf_dp_trace_record_event(enum ADF_DP_TRACE_ID code, uint8_t vdev_id,
		enum adf_proto_type type, enum adf_proto_subtype subtype);
#else
static inline void adf_dp_trace_init(void)
{
}

static inline void adf_dp_trace_set_value(uint8_t proto_bitmap,
		uint8_t no_of_records, uint8_t verbosity)
{
}

static inline void adf_dp_trace_set_track(adf_nbuf_t nbuf,
	enum adf_proto_dir dir)
{
}

static inline void adf_dp_trace(adf_nbuf_t nbuf,
		enum ADF_DP_TRACE_ID code, uint8_t *data, uint8_t size,
		enum adf_proto_dir dir)
{
}

static inline void adf_dp_trace_dump_all(uint32_t count)
{
}

static inline void adf_dp_display_record(struct adf_dp_trace_record_s *record,
							uint16_t index)
{
}

static inline void adf_dp_trace_ptr(adf_nbuf_t nbuf, enum ADF_DP_TRACE_ID code,
		uint8_t *data, uint8_t size, uint16_t msdu_id, uint16_t status)
{
}

static inline void
adf_dp_display_ptr_record(struct adf_dp_trace_record_s *pRecord,
				uint16_t recIndex)
{
}

static inline uint8_t adf_dp_get_proto_bitmap(void)
{
    return 0;
}

static inline void
adf_dp_trace_proto_pkt(enum ADF_DP_TRACE_ID code, uint8_t vdev_id,
		uint8_t *sa, uint8_t *da, enum adf_proto_type type,
		enum adf_proto_subtype subtype, enum adf_proto_dir dir)
{
}

static inline void
adf_dp_display_proto_pkt(struct adf_dp_trace_record_s *record,
				uint16_t index)
{
}

static inline void adf_dp_trace_log_pkt(uint8_t session_id, struct sk_buff *skb,
				enum adf_proto_dir dir)
{
}

static inline
void adf_dp_trace_enable_live_mode(void)
{
}

static inline
void adf_dp_trace_clear_buffer(void)
{
}
#endif

#endif  /* __ADF_TRACE_H */

