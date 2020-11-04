/** @file moal_cfg80211_util.h
 *
 * @brief This file contains the CFG80211 vendor specific defines.
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

#ifndef _MOAL_CFGVENDOR_H_
#define _MOAL_CFGVENDOR_H_

#include "moal_main.h"

#if KERNEL_VERSION(3, 14, 0) <= CFG80211_VERSION_CODE
#define RING_NAME_MAX 32
typedef int wifi_ring_buffer_id;

#define VALID_RING(id) (id >= 0 && id < RING_ID_MAX)

/** WiFi ring control structure */
typedef struct _wifi_ring_ctrl {
	/** Written bytes */
	t_u32 written_bytes;
	/** Read Bytes */
	t_u32 read_bytes;
	/** Written records */
	t_u32 written_records;
} ring_buffer_ctrl;

enum ring_state {
	/** ring is not initialized*/
	RING_STOP = 0,
	/** ring is live and logging*/
	RING_ACTIVE,
	/** ring is initialized but not logging*/
	RING_SUSPEND,
};

/** WiFi ring buffer sttructure */
typedef struct _wifi_ring_buffer {
	/** Ring ID */
	wifi_ring_buffer_id ring_id;
	/** Ring name */
	t_u8 name[RING_NAME_MAX];
	/** Ring size */
	t_u32 ring_size;
	/** Write pointer */
	t_u32 wp;
	/** Read pointer */
	t_u32 rp;
	/** Log level */
	t_u32 log_level;
	/** Threshold */
	t_u32 threshold;
	/** Ring buffer */
	void *ring_buf;
	/** Lock */
	spinlock_t lock;
	/** Buffer control */
	ring_buffer_ctrl ctrl;
	/** Ring state */
	enum ring_state state;
	/** Delayed work */
	struct delayed_work work;
	/** Interval */
	unsigned long interval;
	/** Moal priv */
	moal_private *priv;
} wifi_ring_buffer;

#define VERBOSE_RING_NAME "verbose"
#define EVENT_RING_NAME "event"

#define DEFAULT_RING_BUFFER_SIZE 1024

#define TLV_LOG_HEADER_LEN 4

#define WIFI_LOGGER_MEMORY_DUMP_SUPPORTED MBIT(0) /* Memory dump of Fw*/
#define WIFI_LOGGER_PER_PACKET_TX_RX_STATUS_SUPPORT MBIT(1) /*PKT status*/
#define WIFI_LOGGER_CONNECT_EVENT_SUPPORTED MBIT(2) /* connectivity event*/
#define WIFI_LOGGER_POWER_EVENT_SUPPORTED MBIT(3) /* Power of driver*/
#define WIFI_LOGGER_WAKE_LOCK_SUPPORTED MBIT(4) /* Wake lock of driver*/
#define WIFI_LOGGER_VERBOSE_SUPPORTED MBIT(5) /*verbose log of Fw*/
#define WIFI_LOGGER_WATCHDOG_TIMER_SUPPORTED                                   \
	MBIT(6) /*monitor the health of Fw*/

/**
 * Parameters of wifi logger events are TLVs
 * Event parameters tags are defined as:
 */
#define WIFI_TAG_VENDOR_SPECIFIC 0 // take a byte stream as parameter
#define WIFI_TAG_BSSID 1 // takes a 6 bytes MAC address as parameter
#define WIFI_TAG_ADDR 2 // takes a 6 bytes MAC address as parameter
#define WIFI_TAG_SSID 3 // takes a 32 bytes SSID address as parameter
#define WIFI_TAG_STATUS 4 // takes an integer as parameter
#define WIFI_TAG_REASON_CODE 14 // take a reason code as per 802.11 as parameter
#define WIFI_TAG_RSSI 21 // take an integer as parameter
#define WIFI_TAG_CHANNEL 22 // take an integer as parameter

#define RING_ENTRY_SIZE (sizeof(wifi_ring_buffer_entry))
#define ENTRY_LENGTH(hdr) (hdr->entry_size + RING_ENTRY_SIZE)
#define READ_AVAIL_SPACE(ring)                                                 \
	(ring->ctrl.written_bytes - ring->ctrl.read_bytes)

enum logger_attributes {
	ATTR_WIFI_LOGGER_INVALID = 0,
	ATTR_WIFI_LOGGER_RING_ID,
	ATTR_WIFI_LOGGER_FLAGS,
	ATTR_WIFI_LOGGER_VERBOSE_LEVEL,
	ATTR_WIFI_LOGGER_MIN_DATA_SIZE,
	ATTR_RING_BUFFER_STATUS,
	ATTR_NUM_RINGS,
	ATTR_WIFI_LOGGER_FEATURE_SET,
	ATTR_WIFI_LOGGER_MAX_INTERVAL_SEC,
	ATTR_RING_BUFFER,
	ATTR_NAME,
	ATTR_MEM_DUMP,
	ATTR_ERR_CODE,
	ATTR_RING_DATA,
	ATTR_WAKE_REASON_STAT,
	ATTR_PACKET_FATE_TX,
	ATTR_PACKET_FATE_RX,
	ATTR_PACKET_FATE_DATA,
	ATTR_FW_DUMP_PATH = 20,
	ATTR_DRV_DUMP_PATH = 21,
	ATTR_WIFI_LOGGER_AFTER_LAST,
	ATTR_WIFI_LOGGER_MAX = ATTR_WIFI_LOGGER_AFTER_LAST - 1
};

/* Below events refer to the wifi_connectivity_event ring and shall be supported
 */
enum { WIFI_EVENT_ASSOCIATION_REQUESTED = 0,
       WIFI_EVENT_AUTH_COMPLETE,
       WIFI_EVENT_ASSOC_COMPLETE,
};

enum {
	/* set for binary entries */
	RING_BUFFER_ENTRY_FLAGS_HAS_BINARY = (1 << (0)),
	/* set if 64 bits timestamp is present */
	RING_BUFFER_ENTRY_FLAGS_HAS_TIMESTAMP = (1 << (1))
};

enum { ENTRY_TYPE_CONNECT_EVENT = 1,
       ENTRY_TYPE_PKT,
       ENTRY_TYPE_WAKE_LOCK,
       ENTRY_TYPE_POWER_EVENT,
       ENTRY_TYPE_DATA };

/** WiFi ring buffer entry structure */
typedef struct {
	/** size of payload excluding the header */
	t_u16 entry_size;
	/** Flags */
	t_u8 flags;
	/** entry type */
	t_u8 type;
	/** present if has_timestamp bit is set. */
	t_u64 timestamp;
} __attribute__((packed)) wifi_ring_buffer_entry;

/** WiFi ring buffer status structure*/
typedef struct _wifi_ring_buffer_status {
	/** Ring name */
	t_u8 name[RING_NAME_MAX];
	/** Flag */
	t_u32 flag;
	/** Ring ID */
	wifi_ring_buffer_id ring_id;
	/** Buffer size */
	t_u32 ring_buffer_byte_size;
	/** Verbose Level */
	t_u32 verbose_level;
	/** Written bytes */
	t_u32 written_bytes;
	/** Read bytes */
	t_u32 read_bytes;
	/** Written records */
	t_u32 written_records;
} wifi_ring_buffer_status;

/** TLV log structure */
typedef struct {
	/** Tag */
	u16 tag;
	/** Length of value*/
	u16 length;
	/** Value */
	u8 value[];
} __attribute__((packed)) tlv_log;

/** WiFi ring buffer driver structure */
typedef struct {
	/** event */
	u16 event;
	/** TLV log structure array */
	tlv_log tlvs[];
	/** separate parameter structure per event to be provided and optional
	 * data the event_data is expected to include an official android part,
	 * with some parameter as transmit rate, num retries, num scan result
	 * found etc... as well, event_data can include a vendor proprietary
	 * part which is understood by the developer only
	 */
} __attribute__((packed)) wifi_ring_buffer_driver_connectivity_event;

/** Assoc logger data structure */
typedef struct _assoc_logger {
	/** vendor specific */
	t_u8 oui[3];
	/** BSSID */
	t_u8 bssid[MLAN_MAC_ADDR_LENGTH];
	/** SSID */
	t_u8 ssid[MLAN_MAX_SSID_LENGTH];
	/** RSSI */
	t_s32 rssi;
	/** Channel */
	t_u32 channel;
} assoc_logger_data;

int woal_ring_event_logger(moal_private *priv, int ring_id,
			   pmlan_event pmevent);

int woal_wake_reason_logger(moal_private *priv,
			    mlan_ds_hs_wakeup_reason wake_reason);

#define MD5_PREFIX_LEN 4
#define MAX_FATE_LOG_LEN 32
#define MAX_FRAME_LEN_ETHERNET 1518
#define MAX_FRAME_LEN_80211_MGMT 2352

/** packet_fate_packet_type */
typedef enum {
	PACKET_TYPE_TX,
	PACKET_TYPE_RX,
} packet_fate_packet_type;

/** packet fate frame_type */
typedef enum {
	FRAME_TYPE_UNKNOWN,
	FRAME_TYPE_ETHERNET_II,
	FRAME_TYPE_80211_MGMT,
} frame_type;

/** wifi_tx_packet_fate */
typedef enum {
	/** Sent over air and ACKed. */
	TX_PKT_FATE_ACKED,

	/** Sent over air but not ACKed. (Normal for broadcast/multicast.) */
	TX_PKT_FATE_SENT,

	/** Queued within firmware, but not yet sent over air. */
	TX_PKT_FATE_FW_QUEUED,

	/** Dropped by firmware as invalid. E.g. bad source address, bad
	 *  checksum, or invalid for current state.
	 */
	TX_PKT_FATE_FW_DROP_INVALID,

	/** Dropped by firmware due to lack of buffer space. */
	TX_PKT_FATE_FW_DROP_NOBUFS,

	/** Dropped by firmware for any other reason. Includes frames that were
	 *  sent by driver to firmware, but unaccounted for by firmware.
	 */
	TX_PKT_FATE_FW_DROP_OTHER,

	/** Queued within driver, not yet sent to firmware. */
	TX_PKT_FATE_DRV_QUEUED,

	/** Dropped by driver as invalid. E.g. bad source address, or invalid
	 *  for current state.
	 */
	TX_PKT_FATE_DRV_DROP_INVALID,

	/** Dropped by driver due to lack of buffer space. */
	TX_PKT_FATE_DRV_DROP_NOBUFS,

	/** Dropped by driver for any other reason. */
	TX_PKT_FATE_DRV_DROP_OTHER,
} wifi_tx_packet_fate;

/** wifi_rx_packet_fate */
typedef enum {
	/** Valid and delivered to network stack (e.g., netif_rx()). */
	RX_PKT_FATE_SUCCESS,

	/** Queued within firmware, but not yet sent to driver. */
	RX_PKT_FATE_FW_QUEUED,

	/** Dropped by firmware due to host-programmable filters. */
	RX_PKT_FATE_FW_DROP_FILTER,

	/** Dropped by firmware as invalid. E.g. bad checksum, decrypt failed,
	 *  or invalid for current state.
	 */
	RX_PKT_FATE_FW_DROP_INVALID,

	/** Dropped by firmware due to lack of buffer space. */
	RX_PKT_FATE_FW_DROP_NOBUFS,

	/** Dropped by firmware for any other reason. */
	RX_PKT_FATE_FW_DROP_OTHER,

	/** Queued within driver, not yet delivered to network stack. */
	RX_PKT_FATE_DRV_QUEUED,

	/** Dropped by driver due to filter rules. */
	RX_PKT_FATE_DRV_DROP_FILTER,

	/** Dropped by driver as invalid. E.g. not permitted in current state.
	 */
	RX_PKT_FATE_DRV_DROP_INVALID,

	/** Dropped by driver due to lack of buffer space. */
	RX_PKT_FATE_DRV_DROP_NOBUFS,

	/** Dropped by driver for any other reason. */
	RX_PKT_FATE_DRV_DROP_OTHER,
} wifi_rx_packet_fate;

/** frame_info_i */
typedef struct {
	/** Payload Type */
	frame_type payload_type;
	/** Driver timestamp in uS */
	u32 driver_timestamp_usec;
	/** FW timestamp in uS */
	u32 firmware_timestamp_usec;
	/** Frame Length */
	u32 frame_len;
} frame_info_i;

/** wifi_tx_report_i */
typedef struct {
	/** MD5 prefix */
	char md5_prefix[MD5_PREFIX_LEN];
	/** TX packet fate */
	wifi_tx_packet_fate fate;
	/** frame information */
	frame_info_i frame_inf;
} wifi_tx_report_i;

/** wifi_rx_report_i */
typedef struct {
	/** MD5 prefix */
	char md5_prefix[MD5_PREFIX_LEN];
	/** TX packet fate */
	wifi_rx_packet_fate fate;
	/** frame information */
	frame_info_i frame_inf;
} wifi_rx_report_i;

/** packet_fate_report_t */
typedef struct packet_fate_report_t {
	union {
		wifi_tx_report_i tx_report_i;
		wifi_rx_report_i rx_report_i;
	} u;
} PACKET_FATE_REPORT;

int woal_packet_fate_monitor(moal_private *priv,
			     packet_fate_packet_type pkt_type, t_u8 fate,
			     frame_type payload_type,
			     t_u32 driver_timestamp_usec,
			     t_u32 firmware_timestamp_usec, t_u8 *data,
			     t_u32 len);

/** =========== Define Copied from apf.h START =========== */
/* Number of memory slots, see ldm/stm instructions. */
#define MEM_ITEMS 16
/* Upon program execution starting some memory slots are prefilled: */
/* 4*([APF_FRAME_HEADER_SIZE]&15) */
#define MEM_OFFSET_IPV4_HEADER_SIZE 13
/* Size of packet in bytes. */
#define MEM_OFFSET_PKT_SIZE 14
/* Age since filter installed in seconds. */
#define MEM_OFFSET_FILTER_AGE 15

/* Leave 0 opcode unused as it's a good indicator of accidental incorrect
 * execution (e.g. data).
 */
/* Load 1 byte from immediate offset, e.g. "ldb R0, [5]" */
#define NXP_LDB_OPCODE 1
/* Load 2 bytes from immediate offset, e.g. "ldh R0, [5]" */
#define NXP_LDH_OPCODE 2
/* Load 4 bytes from immediate offset, e.g. "ldw R0, [5]" */
#define NXP_LDW_OPCODE 3
/* Load 1 byte from immediate offset plus register, e.g. "ldbx R0, [5]R0" */
#define NXP_LDBX_OPCODE 4
/* Load 2 byte from immediate offset plus register, e.g. "ldhx R0, [5]R0" */
#define NXP_LDHX_OPCODE 5
/* Load 4 byte from immediate offset plus register, e.g. "ldwx R0, [5]R0" */
#define NXP_LDWX_OPCODE 6
/* Add, e.g. "add R0,5" */
#define NXP_ADD_OPCODE 7
/* Multiply, e.g. "mul R0,5" */
#define NXP_MUL_OPCODE 8
/* Divide, e.g. "div R0,5" */
#define NXP_DIV_OPCODE 9
/* And, e.g. "and R0,5" */
#define NXP_AND_OPCODE 10
/* Or, e.g. "or R0,5" */
#define NXP_OR_OPCODE 11
/* Left shift, e.g, "sh R0, 5" or "sh R0, -5" (shifts right) */
#define NXP_SH_OPCODE 12
/* Load immediate, e.g. "li R0,5" (immediate encoded as signed value) */
#define NXP_LI_OPCODE 13
/* Unconditional jump, e.g. "jmp label" */
#define NXP_JMP_OPCODE 14
/* Compare equal and branch, e.g. "jeq R0,5,label" */
#define NXP_JEQ_OPCODE 15
/* Compare not equal and branch, e.g. "jne R0,5,label" */
#define NXP_JNE_OPCODE 16
/* Compare greater than and branch, e.g. "jgt R0,5,label" */
#define NXP_JGT_OPCODE 17
/* Compare less than and branch, e.g. "jlt R0,5,label" */
#define NXP_JLT_OPCODE 18
/* Compare any bits set and branch, e.g. "jset R0,5,label" */
#define NXP_JSET_OPCODE 19
/* Compare not equal byte sequence, e.g. "jnebs R0,5,label,0x1122334455" */
#define NXP_JNEBS_OPCODE 20
/* Immediate value is one of *_EXT_OPCODE
 * Extended opcodes. These all have an opcode of EXT_OPCODE
 * and specify the actual opcode in the immediate field.
 */
#define NXP_EXT_OPCODE 21
/* Load from memory, e.g. "ldm R0,5"
 * Values 0-15 represent loading the different memory slots.
 */
#define NXP_LDM_EXT_OPCODE 0
/* Store to memory, e.g. "stm R0,5" *
 * Values 16-31 represent storing to the different memory slots.
 */
#define NXP_STM_EXT_OPCODE 16
/* Not, e.g. "not R0" */
#define NXP_NOT_EXT_OPCODE 32
/* Negate, e.g. "neg R0" */
#define NXP_NEG_EXT_OPCODE 33
/* Swap, e.g. "swap R0,R1" */
#define NXP_SWAP_EXT_OPCODE 34
/* Move, e.g. "move R0,R1" */
#define NXP_MOV_EXT_OPCODE 35

#define GET_OPCODE(i) (((i) >> 3) & 31)
#define GET_REGISTER(i) ((i)&1)
#define GET_IMM_LENGTH(i) (((i) >> 1) & 3)
/** =========== Define Copied from apf.h END =========== */

/** =========== Define Copied from apf_interpreter.h START =========== */
/**
 * Version of APF instruction set processed by accept_packet().
 * Should be returned by wifi_get_packet_filter_info.
 */
#define APF_VERSION 2
/** =========== Define Copied from apf_interpreter.h END =========== */

/** =========== Define Copied from apf_interpreter.c START =========== */
/* Return code indicating "packet" should accepted. */
#define PASS_PKT 1
/* Return code indicating "packet" should be dropped. */
#define DROP_PKT 0
/* If "c" is of an unsigned type, generate a compile warning that gets promoted
 * to an error. This makes bounds checking simpler because ">= 0" can be
 * avoided. Otherwise adding superfluous ">= 0" with unsigned expressions
 * generates compile warnings.
 */
#define ENFORCE_UNSIGNED(c) ((c) == (uint32_t)(c))
/** =========== Define Copied from apf_interpreter.c END =========== */

/** depend on the format of skb->data */
#define APF_FRAME_HEADER_SIZE 14
#define PACKET_FILTER_MAX_LEN 1024

enum { PACKET_FILTER_STATE_INIT = 0,
       PACKET_FILTER_STATE_STOP,
       PACKET_FILTER_STATE_START,
};

enum wifi_attr_packet_filter {
	ATTR_PACKET_FILTER_INVALID = 0,
	ATTR_PACKET_FILTER_TOTAL_LENGTH,
	ATTR_PACKET_FILTER_PROGRAM,
	ATTR_PACKET_FILTER_VERSION,
	ATTR_PACKET_FILTER_MAX_LEN,
	ATTR_PACKET_FILTER_AFTER_LAST,
	ATTR_PACKET_FILTER_MAX = ATTR_PACKET_FILTER_AFTER_LAST - 1
};

/** Packet filter structure */
typedef struct _packet_filter {
	spinlock_t lock;
	t_u8 state;
	t_u8 packet_filter_program[PACKET_FILTER_MAX_LEN];
	t_u8 packet_filter_len;
	t_u32 packet_filter_version;
	t_u32 packet_filter_max_len;
} packet_filter;

int woal_filter_packet(moal_private *priv, t_u8 *data, t_u32 len,
		       t_u32 filter_age);

int woal_init_wifi_hal(moal_private *priv);
int woal_deinit_wifi_hal(moal_private *priv);

#define ATTRIBUTE_U32_LEN (nla_total_size(NLA_HDRLEN + 4))
#define VENDOR_ID_OVERHEAD ATTRIBUTE_U32_LEN
#define VENDOR_SUBCMD_OVERHEAD ATTRIBUTE_U32_LEN
#define VENDOR_DATA_OVERHEAD (nla_total_size(NLA_HDRLEN))

#define VENDOR_REPLY_OVERHEAD                                                  \
	(VENDOR_ID_OVERHEAD + VENDOR_SUBCMD_OVERHEAD + VENDOR_DATA_OVERHEAD)

/* Features Enums*/
#define WLAN_FEATURE_INFRA 0x0001 // Basic infrastructure mode support
#define WLAN_FEATURE_INFRA_5G 0x0002 // 5 GHz Band support
#define WLAN_FEATURE_HOTSPOT 0x0004 // GAS/ANQP support
#define WLAN_FEATURE_P2P 0x0008 // Wifi-Direct/P2P
#define WLAN_FEATURE_SOFT_AP 0x0010 // Soft AP support
#define WLAN_FEATURE_GSCAN 0x0020 // Google-Scan APIsi support
#define WLAN_FEATURE_NAN 0x0040 // Neighbor Awareness Networking (NAN)
#define WLAN_FEATURE_D2D_RTT 0x0080 // Device-to-device RTT support
#define WLAN_FEATURE_D2AP_RTT 0x0100 // Device-to-AP RTT support
#define WLAN_FEATURE_BATCH_SCAN 0x0200 // Batched Scan (legacy) support
#define WLAN_FEATURE_PNO 0x0400 // Preferred network offload support
#define WLAN_FEATURE_ADDITIONAL_STA 0x0800 // Two STAs support
#define WLAN_FEATURE_TDLS 0x1000 // Tunnel directed link setup (TDLS)
#define WLAN_FEATURE_TDLS_OFFCHANNEL 0x2000 // TDLS off channel support
#define WLAN_FEATURE_EPR 0x4000 // Enhanced power reporting support
#define WLAN_FEATURE_AP_STA 0x8000 // AP STA Concurrency support
#define WLAN_FEATURE_LINK_LAYER_STATS                                          \
	0x10000 // Link layer stats collection support
#define WLAN_FEATURE_LOGGER 0x20000 // WiFi Logger support
#define WLAN_FEATURE_HAL_EPNO 0x40000 // WiFi enhanced PNO support
#define WLAN_FEATURE_RSSI_MONITOR 0x80000 // RSSI Monitor support
#define WLAN_FEATURE_MKEEP_ALIVE 0x100000 // WiFi mkeep_alive support
#define WLAN_FEATURE_CONFIG_NDO 0x200000 // ND offload configure support
#define WLAN_FEATURE_TX_TRANSMIT_POWER                                         \
	0x400000 // Capture Tx transmit power levels
#define WLAN_FEATURE_CONTROL_ROAMING 0x800000 // Enable/Disable firmware roaming
#define WLAN_FEATURE_IE_WHITELIST 0x1000000 // Probe IE white listing support
#define WLAN_FEATURE_SCAN_RAND                                                 \
	0x2000000 // MAC & Probe Sequence Number randomization Support
// Add more features here

#define MAX_CHANNEL_NUM 200

/** Wifi Band */
typedef enum {
	WIFI_BAND_UNSPECIFIED,
	/** 2.4 GHz */
	WIFI_BAND_BG = 1,
	/** 5 GHz without DFS */
	WIFI_BAND_A = 2,
	/** 5 GHz DFS only */
	WIFI_BAND_A_DFS = 4,
	/** 5 GHz with DFS */
	WIFI_BAND_A_WITH_DFS = 6,
	/** 2.4 GHz + 5 GHz; no DFS */
	WIFI_BAND_ABG = 3,
	/** 2.4 GHz + 5 GHz with DFS */
	WIFI_BAND_ABG_WITH_DFS = 7,

	/** Keep it last */
	WIFI_BAND_LAST,
	WIFI_BAND_MAX = WIFI_BAND_LAST - 1,
} wifi_band;

typedef enum wifi_attr {
	ATTR_FEATURE_SET_INVALID = 0,
	ATTR_SCAN_MAC_OUI_SET = 1,
	ATTR_FEATURE_SET = 2,
	ATTR_NODFS_VALUE = 3,
	ATTR_COUNTRY_CODE = 4,
	ATTR_CHANNELS_BAND = 5,
	ATTR_NUM_CHANNELS = 6,
	ATTR_CHANNEL_LIST = 7,
	ATTR_GET_CONCURRENCY_MATRIX_SET_SIZE_MAX = 8,
	ATTR_GET_CONCURRENCY_MATRIX_SET_SIZE = 9,
	ATTR_GET_CONCURRENCY_MATRIX_SET = 10,
	ATTR_WIFI_AFTER_LAST,
	ATTR_WIFI_MAX = ATTR_WIFI_AFTER_LAST - 1
} wifi_attr_t;

enum mrvl_wlan_vendor_attr_wifi_logger {
	MRVL_WLAN_VENDOR_ATTR_NAME = 10,
};

/**vendor event*/
enum vendor_event {
	event_hang = 0,
	event_rssi_monitor = 0x1501,
	event_cloud_keep_alive = 0x10003,
	event_dfs_radar_detected = 0x10004,
	event_dfs_cac_started = 0x10005,
	event_dfs_cac_finished = 0x10006,
	event_dfs_cac_aborted = 0x10007,
	event_dfs_nop_finished = 0x10008,
	event_wifi_logger_ring_buffer_data = 0x1000b,
	event_wifi_logger_alert,
	event_packet_fate_monitor,
	event_wake_reason_report,
	event_max,
};

/** struct dfs_event */
typedef struct _dfs_event {
	/** Frequency */
	int freq;
	/** HT enable */
	int ht_enabled;
	/** Channel Offset */
	int chan_offset;
	/** Channel width */
	enum nl80211_chan_width chan_width;
	/** Center Frequency 1 */
	int cf1;
	/** Center Frequency 2 */
	int cf2;
} dfs_event;

void woal_cfg80211_dfs_vendor_event(moal_private *priv, int event,
				    struct cfg80211_chan_def *chandef);

enum ATTR_LINK_LAYER_STAT {
	ATTR_LL_STATS_INVALID,
	ATTR_LL_STATS_MPDU_SIZE_THRESHOLD,
	ATTR_LL_STATS_AGGRESSIVE_STATS_GATHERING,
	ATTR_LL_STATS_IFACE,
	ATTR_LL_STATS_NUM_RADIO,
	ATTR_LL_STATS_RADIO,
	ATTR_LL_STATS_CLEAR_REQ_MASK,
	ATTR_LL_STATS_STOP_REQ,
	ATTR_LL_STATS_CLEAR_RSP_MASK,
	ATTR_LL_STATS_STOP_RSP,
	ATTR_LL_STATS_AFTER_LAST,
	ATTR_LL_STATS_MAX = ATTR_LL_STATS_AFTER_LAST - 1,
};

enum ATTR_RSSI_MONITOR {
	ATTR_RSSI_MONITOR_INVALID,
	ATTR_RSSI_MONITOR_CONTROL,
	ATTR_RSSI_MONITOR_MIN_RSSI,
	ATTR_RSSI_MONITOR_MAX_RSSI,
	ATTR_RSSI_MONITOR_CUR_BSSID,
	ATTR_RSSI_MONITOR_CUR_RSSI,
	ATTR_RSSI_MONITOR_AFTER_LAST,
	ATTR_RSSI_MONITOR_MAX = ATTR_RSSI_MONITOR_AFTER_LAST - 1,
};
void woal_cfg80211_rssi_monitor_event(moal_private *priv, t_s16 rssi);

/**vendor sub command*/
enum vendor_sub_command {
	sub_cmd_set_drvdbg = 0,
	sub_cmd_start_keep_alive = 0x0003,
	sub_cmd_stop_keep_alive = 0x0004,
	sub_cmd_dfs_capability = 0x0005,
	sub_cmd_set_scan_mac_oui = 0x0007,
	sub_cmd_set_packet_filter = 0x0011,
	sub_cmd_get_packet_filter_capability,
	sub_cmd_nd_offload = 0x0100,
	sub_cmd_link_statistic_set = 0x1200,
	sub_cmd_link_statistic_get = 0x1201,
	sub_cmd_link_statistic_clr = 0x1202,
	sub_cmd_get_valid_channels = 0x1009,
	sub_cmd_get_wifi_supp_feature_set = 0x100a,
	sub_cmd_set_country_code = 0x100d,
	sub_cmd_get_fw_version = 0x1404,
	sub_cmd_get_drv_version = 0x1406,
	sub_cmd_start_logging = 0x1400,
	sub_cmd_get_wifi_logger_supp_feature_set,
	sub_cmd_get_ring_buff_data,
	sub_cmd_get_ring_buff_status,
	sub_cmd_get_fw_mem_dump = 0x1405,
	sub_cmd_get_drv_mem_dump = 0x1407,
	sub_cmd_start_packet_fate_monitor = 0x1408,
	sub_cmd_rssi_monitor = 0x1500,
	/*Sub-command for wifi hal*/
	sub_cmd_get_roaming_capability = 0x1700,
	sub_cmd_fw_roaming_enable = 0x1701,
	sub_cmd_fw_roaming_config = 0x1702,
	sub_cmd_max,
};

void woal_register_cfg80211_vendor_command(struct wiphy *wiphy);
int woal_cfg80211_vendor_event(moal_private *priv, int event, t_u8 *data,
			       int len);

enum mrvl_wlan_vendor_attr {
	MRVL_WLAN_VENDOR_ATTR_INVALID = 0,
	/* Used by MRVL_NL80211_VENDOR_SUBCMD_DFS_CAPABILITY */
	MRVL_WLAN_VENDOR_ATTR_DFS = 1,
	MRVL_WLAN_VENDOR_ATTR_AFTER_LAST,

	MRVL_WLAN_VENDOR_ATTR_MAX = MRVL_WLAN_VENDOR_ATTR_AFTER_LAST - 1,
};

typedef enum {
	ATTR_ND_OFFLOAD_INVALID = 0,
	ATTR_ND_OFFLOAD_CONTROL,
	ATTR_ND_OFFLOAD_AFTER_LAST,
	ATTR_ND_OFFLOAD_MAX = ATTR_ND_OFFLOAD_AFTER_LAST - 1,
} ND_OFFLOAD_ATTR;

#define MKEEP_ALIVE_IP_PKT_MAX 256
enum mkeep_alive_attributes {
	MKEEP_ALIVE_ATTRIBUTE_INVALID = 0,
	MKEEP_ALIVE_ATTRIBUTE_ID,
	MKEEP_ALIVE_ATTRIBUTE_ETHER_TYPE,
	MKEEP_ALIVE_ATTRIBUTE_IP_PKT,
	MKEEP_ALIVE_ATTRIBUTE_IP_PKT_LEN,
	MKEEP_ALIVE_ATTRIBUTE_SRC_MAC_ADDR,
	MKEEP_ALIVE_ATTRIBUTE_DST_MAC_ADDR,
	MKEEP_ALIVE_ATTRIBUTE_PERIOD_MSEC,
	MKEEP_ALIVE_ATTRIBUTE_RETRY_INTERVAL,
	MKEEP_ALIVE_ATTRIBUTE_RETRY_CNT,
	MKEEP_ALIVE_ATTRIBUTE_AFTER_LAST,
	MKEEP_ALIVE_ATTRIBUTE_MAX = MKEEP_ALIVE_ATTRIBUTE_AFTER_LAST - 1
};

/** WiFi roaming capabilities structure */
typedef struct {
	/** max blacklist size */
	u32 max_blacklist_size;
	/** max whitelist size */
	u32 max_whitelist_size;
} wifi_roaming_capabilities;

/** WiFi BSSID params structure */
typedef struct {
	/** Num of BSSID */
	u32 num_bssid;
	/** List of AP mac address */
	t_u8 mac_addr[MAX_AP_LIST][MLAN_MAC_ADDR_LENGTH];
} wifi_bssid_params;

/** SSID structure */
typedef struct {
	/** Length */
	u32 length;
	/** SSID */
	char ssid[MLAN_MAX_SSID_LENGTH];
} ssid_t;

/** WiFi SSID params structure */
typedef struct {
	/** No of SSID */
	u32 num_ssid;
	/** Whitelist SSID */
	ssid_t whitelist_ssid[MAX_SSID_NUM];
} wifi_ssid_params;

/*Attribute for wifi hal*/
enum mrvl_wlan_vendor_attr_fw_roaming {
	MRVL_WLAN_VENDOR_ATTR_FW_ROAMING_INVALID = 0,
	MRVL_WLAN_VENDOR_ATTR_FW_ROAMING_CAPA,
	MRVL_WLAN_VENDOR_ATTR_FW_ROAMING_CONTROL,
	MRVL_WLAN_VENDOR_ATTR_FW_ROAMING_CONFIG_BSSID,
	MRVL_WLAN_VENDOR_ATTR_FW_ROAMING_CONFIG_SSID,
	/* keep last */
	MRVL_WLAN_VENDOR_ATTR_FW_ROAMING_AFTER_LAST,
	MRVL_WLAN_VENDOR_ATTR_FW_ROAMING_MAX =
		MRVL_WLAN_VENDOR_ATTR_FW_ROAMING_AFTER_LAST - 1
};

#endif
#endif /* _MOAL_CFGVENDOR_H_ */
