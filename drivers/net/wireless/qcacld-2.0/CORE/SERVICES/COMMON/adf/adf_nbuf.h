/*
 * Copyright (c) 2013-2014, 2016-2018 The Linux Foundation. All rights reserved.
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
 * @defgroup adf_nbuf_public network buffer API
 */

/**
 * @ingroup adf_nbuf_public
 * @file adf_nbuf.h
 * This file defines the network buffer abstraction.
 */

#ifndef _ADF_NBUF_H
#define _ADF_NBUF_H
#include <adf_os_util.h>
#include <adf_os_types.h>
#include <adf_os_dma.h>
#include <adf_net_types.h>
#include <adf_os_lock.h>
#include <adf_nbuf_pvt.h>

#ifdef IPA_OFFLOAD
#define IPA_NBUF_OWNER_ID 0xaa55aa55
#endif

#define NBUF_PKT_TRAC_TYPE_EAPOL   0x02
#define NBUF_PKT_TRAC_TYPE_DHCP    0x04
#define NBUF_PKT_TRAC_TYPE_MGMT_ACTION    0x08
#define NBUF_PKT_TRAC_TYPE_ARP     0x10
#define NBUF_PKT_TRAC_TYPE_NS      0x20
#define NBUF_PKT_TRAC_TYPE_NA      0x40
#define NBUF_PKT_TRAC_MAX_STRING   12
#define NBUF_PKT_TRAC_PROTO_STRING 4
#define ADF_NBUF_PKT_ERROR         1
#define ADF_NBUF_FWD_FLAG          1

#define ADF_NBUF_TRAC_IPV4_OFFSET       14
#define ADF_NBUF_TRAC_IPV4_HEADER_SIZE  20
#define ADF_NBUF_TRAC_DHCP_SRV_PORT     67
#define ADF_NBUF_TRAC_DHCP_CLI_PORT     68
#define ADF_NBUF_TRAC_ETH_TYPE_OFFSET   12
#define ADF_NBUF_TRAC_EAPOL_ETH_TYPE    0x888E
#define ADF_NBUF_TRAC_ARP_ETH_TYPE      0x0806
#define ADF_NBUF_TRAC_IPV4_ETH_TYPE     0x0800
#define ADF_NBUF_TRAC_IPV6_ETH_TYPE     0x86dd
#define ADF_NBUF_DEST_MAC_OFFSET        0
#define ADF_NBUF_SRC_MAC_OFFSET         6
#define ADF_NBUF_TRAC_IPV4_PROTO_TYPE_OFFSET  23
#define ADF_NBUF_TRAC_IPV4_DEST_ADDR_OFFSET   30
#define ADF_NBUF_TRAC_IPV4_SRC_ADDR_OFFSET    26
#define ADF_NBUF_TRAC_IPV6_PROTO_TYPE_OFFSET  20
#define ADF_NBUF_TRAC_IPV6_DEST_ADDR_OFFSET   38
#define ADF_NBUF_TRAC_IPV6_DEST_ADDR          0xFF00
#define ADF_NBUF_TRAC_ICMP_TYPE         1
#define ADF_NBUF_TRAC_TCP_TYPE          6
#define ADF_NBUF_TRAC_UDP_TYPE          17
#define ADF_NBUF_TRAC_ICMPV6_TYPE       0x3a
#define ADF_NBUF_TRAC_WAI_ETH_TYPE      0x88b4

#define ADF_NBUF_TRAC_TCP_FLAGS_OFFSET       47
#define ADF_NBUF_TRAC_TCP_ACK_OFFSET         42
#define ADF_NBUF_TRAC_TCP_HEADER_LEN_OFFSET  46
#define ADF_NBUF_TRAC_TCP_ACK_MASK           0x10
#define ADF_NBUF_TRAC_TCP_SPORT_OFFSET       34
#define ADF_NBUF_TRAC_TCP_DPORT_OFFSET       36


/* EAPOL Related MASK */
#define EAPOL_PACKET_TYPE_OFFSET        15
#define EAPOL_KEY_INFO_OFFSET           19
#define EAPOL_PKT_LEN_OFFSET            16
#define EAPOL_KEY_LEN_OFFSET            21
#define EAPOL_MASK                      0x8013
#define EAPOL_M1_BIT_MASK               0x8000
#define EAPOL_M2_BIT_MASK               0x0001
#define EAPOL_M3_BIT_MASK               0x8013
#define EAPOL_M4_BIT_MASK               0x0003

/* Tracked Packet types */
#define NBUF_TX_PKT_INVALID              0
#define NBUF_TX_PKT_DATA_TRACK           1
#define NBUF_TX_PKT_MGMT_TRACK           2

/* Different Packet states */
#define NBUF_TX_PKT_HDD                  1
#define NBUF_TX_PKT_TXRX_ENQUEUE         2
#define NBUF_TX_PKT_TXRX_DEQUEUE         3
#define NBUF_TX_PKT_TXRX                 4
#define NBUF_TX_PKT_HTT                  5
#define NBUF_TX_PKT_HTC                  6
#define NBUF_TX_PKT_HIF                  7
#define NBUF_TX_PKT_CE                   8
#define NBUF_TX_PKT_FREE                 9
#define NBUF_TX_PKT_STATE_MAX            10

/**
 * struct mon_rx_status - This will have monitor mode rx_status extracted from
 * htt_rx_desc used later to update radiotap information.
 * @tsft: Time Synchronization Function timer
 * @chan: Channel frequency
 * @chan_flags: Bitmap of Channel flags, IEEE80211_CHAN_TURBO,
 *              IEEE80211_CHAN_CCK...
 * @flags: For IEEE80211_RADIOTAP_FLAGS
 * @rate: Rate in terms 500Kbps
 * @ant_signal_db: Rx packet RSSI
 * @nr_ant: Number of Antennas used for streaming
 * @mcs_info: Parsed ht sig info
 * @vht_info: Parsed vht sig info
 */

struct mon_rx_status {
	uint64_t tsft;
	uint16_t chan;
	uint16_t chan_flags;
	uint8_t  flags;
	uint8_t  rate;
	uint8_t  ant_signal_db;
	uint8_t  nr_ant;
	struct mon_rx_mcs_info {
		uint8_t  valid;
		uint32_t mcs: 7,
			 bw: 1,
			 smoothing: 1,
			 not_sounding: 1,
			 aggregation: 1,
			 stbc: 2,
			 fec: 1,
			 sgi: 1,
			 ness: 2,
			 reserved: 15;
	} mcs_info;

	struct mon_rx_vht_info {
		uint8_t  valid;
		uint32_t bw: 2,
			 stbc: 1,
			 gid: 6,
			 nss: 3,
			 paid: 9,
			 txps_forbidden: 1,
			 sgi: 1,
			 sgi_disambiguation: 1,
			 coding: 1,
			 ldpc_extra_symbol: 1,
			 mcs: 4,
			 beamformed: 1,
			 reserved: 1;
	} vht_info;
};

/* DHCP Related Mask */
#define DHCP_OPTION53                 (0x35)
#define DHCP_OPTION53_LENGTH          (1)
#define DHCP_OPTION53_OFFSET          (0x11A)
#define DHCP_OPTION53_LENGTH_OFFSET   (0x11B)
#define DHCP_OPTION53_STATUS_OFFSET   (0x11C)
#define DHCP_PKT_LEN_OFFSET           16
#define DHCP_TRANSACTION_ID_OFFSET    46
#define DHCPDISCOVER                  (1)
#define DHCPOFFER                     (2)
#define DHCPREQUEST                   (3)
#define DHCPDECLINE                   (4)
#define DHCPACK                       (5)
#define DHCPNAK                       (6)
#define DHCPRELEASE                   (7)
#define DHCPINFORM                    (8)

/* ARP Related Mask */
#define ARP_SUB_TYPE_OFFSET           20
#define ARP_REQUEST                   (1)
#define ARP_RESPONSE                  (2)

/* IPV4 Related Mask */
#define IPV4_PKT_LEN_OFFSET           16
#define IPV4_TCP_SEQ_NUM_OFFSET       38
#define IPV4_SRC_PORT_OFFSET          34
#define IPV4_DST_PORT_OFFSET          36

/* IPV4 ICMP Related Mask */
#define ICMP_SEQ_NUM_OFFSET           40
#define ICMP_SUBTYPE_OFFSET           34
#define ICMP_REQUEST                  0x08
#define ICMP_RESPONSE                 0x00

/* IPV6 Related Mask */
#define IPV6_PKT_LEN_OFFSET           18
#define IPV6_TCP_SEQ_NUM_OFFSET       58
#define IPV6_SRC_PORT_OFFSET          54
#define IPV6_DST_PORT_OFFSET          56

/* IPV6 ICMPV6 Related Mask */
#define ICMPV6_SEQ_NUM_OFFSET         60
#define ICMPV6_SUBTYPE_OFFSET         54
#define ICMPV6_REQUEST                0x80
#define ICMPV6_RESPONSE               0x81
#define ICMPV6_RS                     0x85
#define ICMPV6_RA                     0x86
#define ICMPV6_NS                     0x87
#define ICMPV6_NA                     0x88
#define ADF_NBUF_IPA_CHECK_MASK       0x80000000

/**
 * adf_proto_type - protocol type
 * @ADF_PROTO_TYPE_DHCP - DHCP
 * @ADF_PROTO_TYPE_EAPOL - EAPOL
 * @ADF_PROTO_TYPE_ARP - ARP
 * @ADF_PROTO_TYPE_MGMT - MGMT
 * @ADF_PROTO_TYPE_EVENT - EVENT
 */
enum adf_proto_type {
	ADF_PROTO_TYPE_DHCP = 0,
	ADF_PROTO_TYPE_EAPOL,
	ADF_PROTO_TYPE_ARP,
	ADF_PROTO_TYPE_MGMT,
	ADF_PROTO_TYPE_EVENT,
	ADF_PROTO_TYPE_MAX
};

/**
 * adf_proto_subtype - subtype of packet
 * @ADF_PROTO_EAPOL_M1 - EAPOL 1/4
 * @ADF_PROTO_EAPOL_M2 - EAPOL 2/4
 * @ADF_PROTO_EAPOL_M3 - EAPOL 3/4
 * @ADF_PROTO_EAPOL_M4 - EAPOL 4/4
 * @ADF_PROTO_DHCP_DISCOVER - discover
 * @ADF_PROTO_DHCP_REQUEST - request
 * @ADF_PROTO_DHCP_OFFER - offer
 * @ADF_PROTO_DHCP_ACK - ACK
 * @ADF_PROTO_DHCP_NACK - NACK
 * @ADF_PROTO_DHCP_RELEASE - release
 * @ADF_PROTO_DHCP_INFORM - inform
 * @ADF_PROTO_DHCP_DECLINE - decline
 * @ADF_PROTO_ARP_REQ - arp request
 * @ADF_PROTO_ARP_RES - arp response
 * @ADF_PROTO_ICMP_REQ - icmp request
 * @ADF_PROTO_ICMP_RES - icmp response
 * @ADF_PROTO_ICMPV6_REQ - icmpv6 request
 * @ADF_PROTO_ICMPV6_RES - icmpv6 response
 * @ADF_PROTO_ICMPV6_RS - icmpv6 rs packet
 * @ADF_PROTO_ICMPV6_RA - icmpv6 ra packet
 * @ADF_PROTO_ICMPV6_NS - icmpv6 ns packet
 * @ADF_PROTO_ICMPV6_NA - icmpv6 na packet
 * @ADF_PROTO_IPV4_UDP - ipv4 udp
 * @ADF_PROTO_IPV4_TCP - ipv4 tcp
 * @ADF_PROTO_IPV6_UDP - ipv6 udp
 * @ADF_PROTO_IPV6_TCP - ipv6 tcp
 * @ADF_PROTO_MGMT_ASSOC -assoc
 * @ADF_PROTO_MGMT_DISASSOC - disassoc
 * @ADF_PROTO_MGMT_AUTH - auth
 * @ADF_PROTO_MGMT_DEAUTH - deauth
 * @ADF_ROAM_SYNCH - roam synch indication from fw
 * @ADF_ROAM_COMPLETE - roam complete cmd to fw
 * @ADF_ROAM_EVENTID - roam eventid from fw
 */
enum adf_proto_subtype {
	ADF_PROTO_INVALID = 0,
	ADF_PROTO_EAPOL_M1,
	ADF_PROTO_EAPOL_M2,
	ADF_PROTO_EAPOL_M3,
	ADF_PROTO_EAPOL_M4,
	ADF_PROTO_DHCP_DISCOVER,
	ADF_PROTO_DHCP_REQUEST,
	ADF_PROTO_DHCP_OFFER,
	ADF_PROTO_DHCP_ACK,
	ADF_PROTO_DHCP_NACK,
	ADF_PROTO_DHCP_RELEASE,
	ADF_PROTO_DHCP_INFORM,
	ADF_PROTO_DHCP_DECLINE,
	ADF_PROTO_ARP_REQ,
	ADF_PROTO_ARP_RES,
	ADF_PROTO_ICMP_REQ,
	ADF_PROTO_ICMP_RES,
	ADF_PROTO_ICMPV6_REQ,
	ADF_PROTO_ICMPV6_RES,
	ADF_PROTO_ICMPV6_RS,
	ADF_PROTO_ICMPV6_RA,
	ADF_PROTO_ICMPV6_NS,
	ADF_PROTO_ICMPV6_NA,
	ADF_PROTO_IPV4_UDP,
	ADF_PROTO_IPV4_TCP,
	ADF_PROTO_IPV6_UDP,
	ADF_PROTO_IPV6_TCP,
	ADF_PROTO_MGMT_ASSOC,
	ADF_PROTO_MGMT_DISASSOC,
	ADF_PROTO_MGMT_AUTH,
	ADF_PROTO_MGMT_DEAUTH,
	ADF_ROAM_SYNCH,
	ADF_ROAM_COMPLETE,
	ADF_ROAM_EVENTID,
	ADF_PROTO_SUBTYPE_MAX
};


/**
 * @brief Platform indepedent packet abstraction
 */
typedef __adf_nbuf_t         adf_nbuf_t;

/**
 * @brief Dma map callback prototype
 */
typedef void (*adf_os_dma_map_cb_t)(void *arg, adf_nbuf_t buf,
                                    adf_os_dma_map_t dmap);

/**
 * @brief invalid handle
 */
#define ADF_NBUF_NULL   __ADF_NBUF_NULL
/**
 * @brief Platform independent packet queue abstraction
 */
typedef __adf_nbuf_queue_t   adf_nbuf_queue_t;

/**
 * BUS/DMA mapping routines
 */

/**
 * @brief Create a DMA map. This can later be used to map
 *        networking buffers. They :
 *          - need space in adf_drv's software descriptor
 *          - are typically created during adf_drv_create
 *          - need to be created before any API(adf_nbuf_map) that uses them
 *
 * @param[in]  osdev os device
 * @param[out] dmap  map handle
 *
 * @return status of the operation
 */
static inline a_status_t
adf_nbuf_dmamap_create(adf_os_device_t osdev,
                       adf_os_dma_map_t *dmap)
{
    return (__adf_nbuf_dmamap_create(osdev, dmap));
}


/**
 * @brief Delete a dmap map
 *
 * @param[in] osdev os device
 * @param[in] dmap
 */
static inline void
adf_nbuf_dmamap_destroy(adf_os_device_t osdev, adf_os_dma_map_t dmap)
{
    __adf_nbuf_dmamap_destroy(osdev, dmap);
}

/**
 * @brief Setup the map callback for a dma map
 *
 * @param[in] map DMA map
 * @param[in] cb  callback function
 * @param[in] arg context for callback function
 */
static inline void
adf_nbuf_dmamap_set_cb(adf_os_dma_map_t dmap, adf_os_dma_map_cb_t cb,
                       void *arg)
{
    __adf_nbuf_dmamap_set_cb(dmap, cb, arg);
}

/**
 * @brief Map a buffer to local bus address space
 *
 * @param[in] osdev  os device
 * @param[in] buf    buf to be mapped
 *                   (mapping info is stored in the buf's meta-data area)
 * @param[in] dir    DMA direction
 *
 * @return status of the operation
 */
static inline a_status_t
adf_nbuf_map(adf_os_device_t        osdev,
             adf_nbuf_t             buf,
             adf_os_dma_dir_t       dir)
{
#if defined(HIF_PCI)
    return __adf_nbuf_map(osdev, buf, dir);
#else
    return 0;
#endif
}


/**
 * @brief Unmap a previously mapped buf
 *
 * @param[in] osdev  os device
 * @param[in] buf    buf to be unmapped
 *                   (mapping info is stored in the buf's meta-data area)
 * @param[in] dir    DMA direction
 */
static inline void
adf_nbuf_unmap(adf_os_device_t      osdev,
               adf_nbuf_t           buf,
               adf_os_dma_dir_t     dir)
{
#if defined(HIF_PCI)
    __adf_nbuf_unmap(osdev, buf, dir);
#endif
}

static inline a_status_t
adf_nbuf_map_single(
    adf_os_device_t osdev, adf_nbuf_t buf, adf_os_dma_dir_t dir)
{
#if defined(HIF_PCI)
    return __adf_nbuf_map_single(osdev, buf, dir);
#else
    return 0;
#endif
}

static inline void
adf_nbuf_unmap_single(
    adf_os_device_t osdev, adf_nbuf_t buf, adf_os_dma_dir_t dir)
{
#if defined(HIF_PCI)
    __adf_nbuf_unmap_single(osdev, buf, dir);
#endif
}

static inline int
adf_nbuf_get_num_frags(adf_nbuf_t buf)
{
    return __adf_nbuf_get_num_frags(buf);
}

static inline int
adf_nbuf_get_frag_len(adf_nbuf_t buf, int frag_num)
{
    return __adf_nbuf_get_frag_len(buf, frag_num);
}

static inline unsigned char *
adf_nbuf_get_frag_vaddr(adf_nbuf_t buf, int frag_num)
{
    return __adf_nbuf_get_frag_vaddr(buf, frag_num);
}

static inline a_uint32_t
adf_nbuf_get_frag_paddr_lo(adf_nbuf_t buf, int frag_num)
{
    return __adf_nbuf_get_frag_paddr_lo(buf, frag_num);
}

static inline int
adf_nbuf_get_frag_is_wordstream(adf_nbuf_t buf, int frag_num)
{
    return __adf_nbuf_get_frag_is_wordstream(buf, frag_num);
}

static inline void
adf_nbuf_set_frag_is_wordstream(adf_nbuf_t buf, int frag_num, int is_wordstream)
{
    __adf_nbuf_set_frag_is_wordstream(buf, frag_num, is_wordstream);
}

static inline void
adf_nbuf_frag_push_head(
    adf_nbuf_t buf,
    int frag_len,
    char *frag_vaddr,
    u_int32_t frag_paddr_lo,
    u_int32_t frag_paddr_hi)
{
    __adf_nbuf_frag_push_head(
        buf, frag_len, frag_vaddr, frag_paddr_lo, frag_paddr_hi);
}

/**
 * @brief returns information about the mapped buf
 *
 * @param[in]  bmap map handle
 * @param[out] sg   map info
 */
static inline void
adf_nbuf_dmamap_info(adf_os_dma_map_t bmap, adf_os_dmamap_info_t *sg)
{
    __adf_nbuf_dmamap_info(bmap, sg);
}

#ifdef MEMORY_DEBUG
void adf_net_buf_debug_init(void);
void adf_net_buf_debug_exit(void);
void adf_net_buf_debug_clean(void);
void adf_net_buf_debug_add_node(adf_nbuf_t net_buf, size_t size,
			uint8_t *file_name, uint32_t line_num);
void adf_net_buf_debug_delete_node(adf_nbuf_t net_buf);
void adf_net_buf_debug_release_skb(adf_nbuf_t net_buf);

/* nbuf allocation routines */

#define adf_nbuf_alloc(d, s, r, a, p)			\
	adf_nbuf_alloc_debug(d, s, r, a, p, __FILE__, __LINE__)
static inline adf_nbuf_t
adf_nbuf_alloc_debug(adf_os_device_t osdev, adf_os_size_t size, int reserve,
		int align, int prio, uint8_t *file_name,
		uint32_t line_num)
{
	adf_nbuf_t net_buf;
	net_buf = __adf_nbuf_alloc(osdev, size, reserve, align, prio);

	/* Store SKB in internal ADF tracking table */
	if (adf_os_likely(net_buf))
		adf_net_buf_debug_add_node(net_buf, size, file_name, line_num);

	return net_buf;
}

static inline void adf_nbuf_free(adf_nbuf_t net_buf)
{
	/* Remove SKB from internal ADF tracking table */
	if (adf_os_likely(net_buf))
		adf_net_buf_debug_delete_node(net_buf);

	__adf_nbuf_free(net_buf);
}

#else

static inline void adf_net_buf_debug_release_skb(adf_nbuf_t net_buf)
{
	return;
}

/*
 * nbuf allocation rouines
 */


/**
 * @brief Allocate adf_nbuf
 *
 * The nbuf created is guarenteed to have only 1 physical segment
 *
 * @param[in] hdl   platform device object
 * @param[in] size  data buffer size for this adf_nbuf including max header
 *                  size
 * @param[in] reserve  headroom to start with.
 * @param[in] align    alignment for the start buffer.
 * @param[i] prio   Indicate if the nbuf is high priority (some OSes e.g darwin
 *                   polls few times if allocation fails and priority is  TRUE)
 *
 * @return The new adf_nbuf instance or NULL if there's not enough memory.
 */
static inline adf_nbuf_t
adf_nbuf_alloc(adf_os_device_t      osdev,
               adf_os_size_t        size,
               int                  reserve,
               int                  align,
               int                  prio)
{
    return __adf_nbuf_alloc(osdev, size, reserve,align, prio);
}

#ifdef QCA_ARP_SPOOFING_WAR
/**
 * adf_rx_nbuf_alloc() Allocate adf_nbuf for Rx packet
 *
 * The nbuf created is guarenteed to have only 1 physical segment
 *
 * @hdl:   platform device object
 * @size:  data buffer size for this adf_nbuf including max header
 *                  size
 * @reserve:  headroom to start with.
 * @align:    alignment for the start buffer.
 * @prio:   Indicate if the nbuf is high priority (some OSes e.g darwin
 *                   polls few times if allocation fails and priority is  TRUE)
 *
 * Return: The new adf_nbuf instance or NULL if there's not enough memory.
 */
static inline adf_nbuf_t
adf_rx_nbuf_alloc(adf_os_device_t      osdev,
               adf_os_size_t        size,
               int                  reserve,
               int                  align,
               int                  prio)
{
    return __adf_rx_nbuf_alloc(osdev, size, reserve,align, prio);
}
#endif

/**
 * @brief Free adf_nbuf
 *
 * @param[in] buf buffer to free
 */
static inline void
adf_nbuf_free(adf_nbuf_t buf)
{
    __adf_nbuf_free(buf);
}

#endif

/**
 * @brief Free adf_nbuf
 *
 * @param[in] buf buffer to free
 */
static inline void
adf_nbuf_ref(adf_nbuf_t buf)
{
    __adf_nbuf_ref(buf);
}

/**
 *  @brief Check whether the buffer is shared
 *  @param skb: buffer to check
 *
 *  Returns true if more than one person has a reference to this
 *  buffer.
 */
static inline int
adf_nbuf_shared(adf_nbuf_t buf)
{
    return __adf_nbuf_shared(buf);
}


/**
 * @brief Free a list of adf_nbufs and tell the OS their tx status (if req'd)
 *
 * @param[in] bufs - list of netbufs to free
 * @param[in] tx_err - whether the tx frames were transmitted successfully
 */
static inline void
adf_nbuf_tx_free(adf_nbuf_t buf_list, int tx_err)
{
    __adf_nbuf_tx_free(buf_list, tx_err);
}

/**
 * @brief Reallocate such that there's required headroom in
 *        buf. Note that this can allocate a new buffer, or
 *        change geometry of the orignial buffer. The new buffer
 *        is returned in the (new_buf).
 *
 * @param[in] buf (older buffer)
 * @param[in] headroom
 *
 * @return newly allocated buffer
 */
static inline adf_nbuf_t
adf_nbuf_realloc_headroom(adf_nbuf_t buf, a_uint32_t headroom)
{
    return (__adf_nbuf_realloc_headroom(buf, headroom));
}


/**
 * @brief expand the tailroom to the new tailroom, but the buffer
 * remains the same
 *
 * @param[in] buf       buffer
 * @param[in] tailroom  new tailroom
 *
 * @return expanded buffer or NULL on failure
 */
static inline adf_nbuf_t
adf_nbuf_realloc_tailroom(adf_nbuf_t buf, a_uint32_t tailroom)
{
    return (__adf_nbuf_realloc_tailroom(buf, tailroom));
}


/**
 * @brief this will expand both tail & head room for a given
 *        buffer, you may or may not get a new buffer.Use it
 *        only when its required to expand both. Otherwise use
 *        realloc (head/tail) will solve the purpose. Reason for
 *        having an extra API is that some OS do this in more
 *        optimized way, rather than calling realloc (head/tail)
 *        back to back.
 *
 * @param[in] buf       buffer
 * @param[in] headroom  new headroom
 * @param[in] tailroom  new tailroom
 *
 * @return expanded buffer
 */
static inline adf_nbuf_t
adf_nbuf_expand(adf_nbuf_t buf, a_uint32_t headroom, a_uint32_t tailroom)
{
    return (__adf_nbuf_expand(buf,headroom,tailroom));
}


/**
 * @brief Copy src buffer into dst. This API is useful, for
 *        example, because most native buffer provide a way to
 *        copy a chain into a single buffer. Therefore as a side
 *        effect, it also "linearizes" a buffer (which is
 *        perhaps why you'll use it mostly). It creates a
 *        writeable copy.
 *
 * @param[in] buf source nbuf to copy from
 *
 * @return the new nbuf
 */
static inline adf_nbuf_t
adf_nbuf_copy(adf_nbuf_t buf)
{
    return(__adf_nbuf_copy(buf));
}


/**
 * @brief link two nbufs, the new buf is piggybacked into the
 *        older one.
 *
 * @param[in] dst   buffer to piggyback into
 * @param[in] src   buffer to put
 *
 * @return status of the call - 0 successful
 */
static inline a_status_t
adf_nbuf_cat(adf_nbuf_t dst, adf_nbuf_t src)
{
    return __adf_nbuf_cat(dst, src);
}


/**
 * @brief return the length of the copy bits for skb
 *
 * @param skb, offset, len, to
 *
 * @return int32_t
 */
static inline int32_t
adf_nbuf_copy_bits(adf_nbuf_t nbuf, u_int32_t offset, u_int32_t len, void *to)
{
    return __adf_nbuf_copy_bits(nbuf, offset, len, to);
}


/**
 * @brief clone the nbuf (copy is readonly)
 *
 * @param[in] buf nbuf to clone from
 *
 * @return cloned buffer
 */
static inline adf_nbuf_t
adf_nbuf_clone(adf_nbuf_t buf)
{
    return(__adf_nbuf_clone(buf));
}


/**
 * @brief  Create a version of the specified nbuf whose
 *         contents can be safely modified without affecting
 *         other users.If the nbuf is a clone then this function
 *         creates a new copy of the data. If the buffer is not
 *         a clone the original buffer is returned.
 *
 * @param[in] buf   source nbuf to create a writable copy from
 *
 * @return new buffer which is writeable
 */
static inline adf_nbuf_t
adf_nbuf_unshare(adf_nbuf_t buf)
{
    return(__adf_nbuf_unshare(buf));
}



/*
 * nbuf manipulation routines
 */

/**
 * @brief  return the address of an nbuf's buffer
 *
 * @param[in] buf netbuf
 *
 * @return head address
 */
static inline a_uint8_t *
adf_nbuf_head(adf_nbuf_t buf)
{
    return __adf_nbuf_head(buf);
}

/**
 * @brief  return the address of the start of data within an nbuf
 *
 * @param[in] buf buffer
 *
 * @return data address
 */
static inline a_uint8_t *
adf_nbuf_data(adf_nbuf_t buf)
{
    return __adf_nbuf_data(buf);
}

/**
 * adf_nbuf_data_addr() - Return the address of skb->data
 * @buf: buffer
 *
 * This function returns the address of skb->data
 *
 * Return: skb->data address
 */
static inline uint8_t *
adf_nbuf_data_addr(adf_nbuf_t buf)
{
	return __adf_nbuf_data_addr(buf);
}

/**
 * @brief return the amount of headroom int the current nbuf
 *
 * @param[in] buf   buffer
 *
 * @return amount of head room
 */
static inline a_uint32_t
adf_nbuf_headroom(adf_nbuf_t buf)
{
    return (__adf_nbuf_headroom(buf));
}


/**
 * @brief return the amount of tail space available
 *
 * @param[in] buf   buffer
 *
 * @return amount of tail room
 */
static inline a_uint32_t
adf_nbuf_tailroom(adf_nbuf_t buf)
{
    return (__adf_nbuf_tailroom(buf));
}


/**
 * @brief Push data in the front
 *
 * @param[in] buf      buf instance
 * @param[in] size     size to be pushed
 *
 * @return New data pointer of this buf after data has been pushed,
 *         or NULL if there is not enough room in this buf.
 */
static inline a_uint8_t *
adf_nbuf_push_head(adf_nbuf_t buf, adf_os_size_t size)
{
    return __adf_nbuf_push_head(buf, size);
}


/**
 * @brief Puts data in the end
 *
 * @param[in] buf      buf instance
 * @param[in] size     size to be pushed
 *
 * @return data pointer of this buf where new data has to be
 *         put, or NULL if there is not enough room in this buf.
 */
static inline a_uint8_t *
adf_nbuf_put_tail(adf_nbuf_t buf, adf_os_size_t size)
{
    return __adf_nbuf_put_tail(buf, size);
}

/**
 * @brief before put buf into pool,turn it to init state
 *
 * @param[in] buf        buf instance
 * @param[in] reserve    headroom to start with.
 * @param[in] align      alignment for the start buffer.
 * @param[in] tail_size  put size to the tail of buffer
 *
 * @return data pointer of this buf where new data has to be
 *         put, or NULL if there is not enough room in this buf.
 */

static inline a_uint8_t *
adf_nbuf_init(adf_nbuf_t buf, adf_os_size_t reverse, adf_os_size_t align, adf_os_size_t tail_size)
{
    return __adf_nbuf_init(buf, reverse, align, tail_size);
}

static inline void
adf_nbuf_free_pool(adf_nbuf_t buf)
{
    __adf_nbuf_free_pool(buf);
}

/**
 * @brief pull data out from the front
 *
 * @param[in] buf   buf instance
 * @param[in] size     size to be popped
 *
 * @return New data pointer of this buf after data has been popped,
 *         or NULL if there is not sufficient data to pull.
 */
static inline a_uint8_t *
adf_nbuf_pull_head(adf_nbuf_t buf, adf_os_size_t size)
{
    return __adf_nbuf_pull_head(buf, size);
}


/**
 *
 * @brief trim data out from the end
 *
 * @param[in] buf   buf instance
 * @param[in] size     size to be popped
 *
 * @return none
 */
static inline void
adf_nbuf_trim_tail(adf_nbuf_t buf, adf_os_size_t size)
{
    __adf_nbuf_trim_tail(buf, size);
}


/**
 * @brief Get the length of the buf
 *
 * @param[in] buf the buf instance
 *
 * @return The total length of this buf.
 */
static inline adf_os_size_t
adf_nbuf_len(adf_nbuf_t buf)
{
    return (__adf_nbuf_len(buf));
}

/**
 * @brief Set the length of the buf
 *
 * @param[in] buf the buf instance
 * @param[in] size to be set
 *
 * @return none
 */
static inline void
adf_nbuf_set_pktlen(adf_nbuf_t buf, uint32_t len)
{
    __adf_nbuf_set_pktlen(buf, len);
}

/**
 * @brief test whether the nbuf is cloned or not
 *
 * @param[in] buf   buffer
 *
 * @return TRUE if it is cloned, else FALSE
 */
static inline a_bool_t
adf_nbuf_is_cloned(adf_nbuf_t buf)
{
    return (__adf_nbuf_is_cloned(buf));
}

/**
 *
 * @brief trim data out from the end
 *
 * @param[in] buf   buf instance
 * @param[in] size     size to be popped
 *
 * @return none
 */
static inline void
adf_nbuf_reserve(adf_nbuf_t buf, adf_os_size_t size)
{
    __adf_nbuf_reserve(buf, size);
}


/*
 * nbuf frag routines
 */

/**
 * @brief return the frag pointer & length of the frag
 *
 * @param[in]  buf   buffer
 * @param[out] sg    this will return all the frags of the nbuf
 *
 */
static inline void
adf_nbuf_frag_info(adf_nbuf_t buf, adf_os_sglist_t *sg)
{
    __adf_nbuf_frag_info(buf, sg);
}
/**
 * @brief return the data pointer & length of the header
 *
 * @param[in]  buf  nbuf
 * @param[out] addr data pointer
 * @param[out] len  length of the data
 *
 */
static inline void
adf_nbuf_peek_header(adf_nbuf_t buf, a_uint8_t **addr, a_uint32_t *len)
{
    __adf_nbuf_peek_header(buf, addr, len);
}
/*
 * nbuf private context routines
 */

/**
 * @brief get the priv pointer from the nbuf'f private space
 *
 * @param[in] buf
 *
 * @return data pointer to typecast into your priv structure
 */
static inline a_uint8_t *
adf_nbuf_get_priv(adf_nbuf_t buf)
{
    return (__adf_nbuf_get_priv(buf));
}


/*
 * nbuf queue routines
 */


/**
 * @brief Initialize buf queue
 *
 * @param[in] head  buf queue head
 */
static inline void
adf_nbuf_queue_init(adf_nbuf_queue_t *head)
{
    __adf_nbuf_queue_init(head);
}


/**
 * @brief Append a nbuf to the tail of the buf queue
 *
 * @param[in] head  buf queue head
 * @param[in] buf   buf
 */
static inline void
adf_nbuf_queue_add(adf_nbuf_queue_t *head, adf_nbuf_t buf)
{
    __adf_nbuf_queue_add(head, buf);
}

/**
 * @brief   Insert nbuf at the head of queue
 *
 * @param[in] head  buf queue head
 * @param[in] buf   buf
 */
static inline void
adf_nbuf_queue_insert_head(adf_nbuf_queue_t *head, adf_nbuf_t buf)
{
    __adf_nbuf_queue_insert_head(head, buf);
}

/**
 * @brief Retrieve a buf from the head of the buf queue
 *
 * @param[in] head    buf queue head
 *
 * @return The head buf in the buf queue.
 */
static inline adf_nbuf_t
adf_nbuf_queue_remove(adf_nbuf_queue_t *head)
{
    return __adf_nbuf_queue_remove(head);
}


/**
 * @brief get the length of the queue
 *
 * @param[in] head  buf queue head
 *
 * @return length of the queue
 */
static inline a_uint32_t
adf_nbuf_queue_len(adf_nbuf_queue_t *head)
{
    return __adf_nbuf_queue_len(head);
}


/**
 * @brief get the first guy/packet in the queue
 *
 * @param[in] head  buf queue head
 *
 * @return first buffer in queue
 */
static inline adf_nbuf_t
adf_nbuf_queue_first(adf_nbuf_queue_t *head)
{
    return (__adf_nbuf_queue_first(head));
}


/**
 * @brief get the next guy/packet of the given buffer (or
 *        packet)
 *
 * @param[in] buf   buffer
 *
 * @return next buffer/packet
 */
static inline adf_nbuf_t
adf_nbuf_queue_next(adf_nbuf_t buf)
{
    return (__adf_nbuf_queue_next(buf));
}


/**
 * @brief Check if the buf queue is empty
 *
 * @param[in] nbq   buf queue handle
 *
 * @return    TRUE  if queue is empty
 * @return    FALSE if queue is not emty
 */
static inline a_bool_t
adf_nbuf_is_queue_empty(adf_nbuf_queue_t * nbq)
{
    return __adf_nbuf_is_queue_empty(nbq);
}


/**
 * @brief get the next packet in the linked list
 * @details
 *  This function can be used when nbufs are directly linked into a list,
 *  rather than using a separate network buffer queue object.
 *
 * @param[in] buf buffer
 *
 * @return next network buffer in the linked list
 */
static inline adf_nbuf_t
adf_nbuf_next(adf_nbuf_t buf)
{
    return __adf_nbuf_next(buf);
}


/**
 * @brief add a packet to a linked list
 * @details
 *  This function can be used to directly link nbufs, rather than using
 *  a separate network buffer queue object.
 *
 * @param[in] this_buf predecessor buffer
 * @param[in] next_buf successor buffer
 */
static inline void
adf_nbuf_set_next(adf_nbuf_t this_buf, adf_nbuf_t next_buf)
{
    __adf_nbuf_set_next(this_buf, next_buf);
}


/*
 * nbuf extension routines XXX
 */

/**
 * @brief link extension of this packet contained in a new nbuf
 * @details
 *  This function is used to link up many nbufs containing a single logical
 *  packet - not a collection of packets. Do not use for linking the first
 *  extension to the head
 * @param[in] this_buf predecessor buffer
 * @param[in] next_buf successor buffer
 */
static inline void
adf_nbuf_set_next_ext(adf_nbuf_t this_buf, adf_nbuf_t next_buf)
{
    __adf_nbuf_set_next_ext(this_buf, next_buf);
}

/**
 * @brief get the next packet extension in the linked list
 * @details
 *
 * @param[in] buf buffer
 *
 * @return next network buffer in the linked list
 */
static inline adf_nbuf_t
adf_nbuf_next_ext(adf_nbuf_t buf)
{
    return __adf_nbuf_next_ext(buf);
}

/**
 * @brief link list of packet extensions to the head segment
 * @details
 *  This function is used to link up a list of packet extensions (seg1, 2,
 *  ...) to the nbuf holding the head segment (seg0)
 * @param[in] head_buf nbuf holding head segment (single)
 * @param[in] ext_list nbuf list holding linked extensions to the head
 * @param[in] ext_len Total length of all buffers in the extension list
 */
static inline void
adf_nbuf_append_ext_list(adf_nbuf_t head_buf, adf_nbuf_t ext_list,
        adf_os_size_t ext_len)
{
    __adf_nbuf_append_ext_list(head_buf, ext_list, ext_len);
}

/**
 * adf_nbuf_get_ext_list() - Get the link to extended nbuf list.
 * @head_buf: Network buf holding head segment (single)
 *
 * This ext_list is populated when we have Jumbo packet, for example in case of
 * monitor mode amsdu packet reception, and are stiched using frags_list.
 *
 * Return: Network buf list holding linked extensions from head buf.
 */
static inline adf_nbuf_t adf_nbuf_get_ext_list(adf_nbuf_t head_buf)
{
	return (adf_nbuf_t)__adf_nbuf_get_ext_list(head_buf);
}

/**
 * @brief Gets the tx checksumming to be performed on this buf
 *
 * @param[in]  buf       buffer
 * @param[out] hdr_off   the (tcp) header start
 * @param[out] where     the checksum offset
 */
static inline adf_net_cksum_type_t
adf_nbuf_tx_cksum_info(adf_nbuf_t buf, a_uint8_t **hdr_off, a_uint8_t **where)
{
    return(__adf_nbuf_tx_cksum_info(buf, hdr_off, where));
}


/**
 * @brief Gets the tx checksum offload demand
 *
 * @param[in]  buf             buffer
 * @return adf_nbuf_tx_cksum_t checksum offload demand for the frame
 */
static inline adf_nbuf_tx_cksum_t
adf_nbuf_get_tx_cksum(adf_nbuf_t buf)
{
    return (__adf_nbuf_get_tx_cksum(buf));
}

/**
 * @brief Drivers that support hw checksumming use this to
 *        indicate checksum info to the stack.
 *
 * @param[in]  buf      buffer
 * @param[in]  cksum    checksum
 */
static inline void
adf_nbuf_set_rx_cksum(adf_nbuf_t buf, adf_nbuf_rx_cksum_t *cksum)
{
    __adf_nbuf_set_rx_cksum(buf, cksum);
}


/**
 * @brief Drivers that are capable of TCP Large segment offload
 *        use this to get the offload info out of an buf.
 *
 * @param[in]  buf  buffer
 * @param[out] tso  offload info
 */
static inline void
adf_nbuf_get_tso_info(adf_nbuf_t buf, adf_nbuf_tso_t *tso)
{
    __adf_nbuf_get_tso_info(buf, tso);
}


/*static inline void
adf_nbuf_set_vlan_info(adf_nbuf_t buf, adf_net_vlan_tag_t vlan_tag)
{
    __adf_nbuf_set_vlan_info(buf, vlan_tag);
}*/

/**
 * @brief This function extracts the vid & priority from an
 *        nbuf
 *
 *
 * @param[in] hdl   net handle
 * @param[in] buf   buffer
 * @param[in] vlan  vlan header
 *
 * @return status of the operation
 */
static inline a_status_t
adf_nbuf_get_vlan_info(adf_net_handle_t hdl, adf_nbuf_t buf,
                       adf_net_vlanhdr_t *vlan)
{
    return __adf_nbuf_get_vlan_info(hdl, buf, vlan);
}

/**
 * @brief This function extracts the TID value from nbuf
 *
 * @param[in] buf   buffer
 *
 * @return TID value
 */
static inline a_uint8_t
adf_nbuf_get_tid(adf_nbuf_t buf)
{
    return __adf_nbuf_get_tid(buf);
}

/**
 * @brief This function sets the TID value in nbuf
 *
 * @param[in] buf   buffer
 *
 * @param[in] tid  TID value
 */
static inline void
adf_nbuf_set_tid(adf_nbuf_t buf, a_uint8_t tid)
{
    __adf_nbuf_set_tid(buf, tid);
}

/**
 * @brief This function extracts the exemption type from nbuf
 *
 * @param[in] buf   buffer
 *
 * @return exemption type
 */
static inline a_uint8_t
adf_nbuf_get_exemption_type(adf_nbuf_t buf)
{
    return __adf_nbuf_get_exemption_type(buf);
}

static inline void
adf_nbuf_reset_ctxt(__adf_nbuf_t nbuf)
{
    __adf_nbuf_reset_ctxt(nbuf);
}

/**
 * @brief This function peeks data into the buffer at given offset
 *
 * @param[in] buf   buffer
 * @param[out] data  peeked output buffer
 * @param[in] off   offset
 * @param[in] len   length of buffer requested beyond offset
 *
 * @return status of operation
 */
static inline a_status_t
adf_nbuf_peek_data(adf_nbuf_t buf, void **data, a_uint32_t off,
		   a_uint32_t len)
{
	return __adf_nbuf_peek_data(buf, data, off, len);
}

/**
 * @brief This function peeks data into the buffer at given offset
 *
 * @param[in] buf   buffer
 * @param[in] proto protocol
 */
static inline void
adf_nbuf_set_protocol(adf_nbuf_t buf, uint16_t proto)
{
	__adf_nbuf_set_protocol(buf, proto);
}

/**
 * @brief This function return packet proto type
 *
 * @param[in] buf    buffer
 */
static inline uint8_t
adf_nbuf_trace_get_proto_type(adf_nbuf_t buf)
{
   return __adf_nbuf_trace_get_proto_type(buf);
}

/**
 * @brief This function updates packet proto type
 *
 * @param[in] buf        buffer
 * @param[in] proto_type protocol type
*/
static inline void
adf_nbuf_trace_set_proto_type(adf_nbuf_t buf, uint8_t proto_type)
{
   __adf_nbuf_trace_set_proto_type(buf, proto_type);
}

/**
 * adf_nbuf_get_fwd_flag() - get packet forwarding flag
 * @buf: pointer to adf_nbuf_t structure
 *
 * Returns: packet forwarding flag
*/
static inline uint8_t
adf_nbuf_get_fwd_flag(adf_nbuf_t buf)
{
   return __adf_nbuf_get_fwd_flag(buf);
}

/**
 * adf_nbuf_get_fwd_flag() - update packet forwarding flag
 * @buf: pointer to adf_nbuf_t structure
 * @flag: forwarding flag
 *
 * Returns: none
*/
static inline void
adf_nbuf_set_fwd_flag(adf_nbuf_t buf, uint8_t flag)
{
   __adf_nbuf_set_fwd_flag(buf, flag);
}

/**
 * adf_nbuf_is_ipa_nbuf() - Check if frame owner is IPA
 * @skb: Pointer to skb
 *
 * Returns: TRUE if the owner is IPA else FALSE
 *
 */
#if (defined(QCA_MDM_DEVICE) && defined(IPA_OFFLOAD))
static inline bool
adf_nbuf_is_ipa_nbuf(adf_nbuf_t buf)
{
    return (NBUF_OWNER_ID(buf) == IPA_NBUF_OWNER_ID);
}
#else
static inline bool
adf_nbuf_is_ipa_nbuf(adf_nbuf_t buf)
{
    return false;
}
#endif /* QCA_MDM_DEVICE && IPA_OFFLOAD*/

/**
 * @brief This function registers protocol trace callback
 *
 * @param[in] adf_nbuf_trace_update_t   callback pointer
 */
static inline void
adf_nbuf_reg_trace_cb(adf_nbuf_trace_update_t cb_func_ptr)
{
   __adf_nbuf_reg_trace_cb(cb_func_ptr);
}

/**
 * @brief This function updates protocol event
 *
 * @param[in] buf      buffer
 * @param[in] char *   event string
 */
static inline void
adf_nbuf_trace_update(adf_nbuf_t buf, char *event_string)
{
    __adf_nbuf_trace_update(buf, event_string);
}

/**
 * @brief This function stores a flag specifying this TX frame
 *        is suitable for downloading though a 2nd TX data pipe
 *        that is used for short frames for protocols that can
 *        accept out-of-order delivery.
 *
 * @param[in] buf        buffer
 * @param[in] candi      candidate of parallel download frame
 */
static inline void
adf_nbuf_set_tx_parallel_dnload_frm(adf_nbuf_t buf, uint8_t candi)
{
   __adf_nbuf_set_tx_htt2_frm(buf, candi);
}

/**
 * @brief This function return whether this TX frame is allow
 *        to download though a 2nd TX data pipe or not.
 *
 * @param[in] buf    buffer
 */
static inline uint8_t
adf_nbuf_get_tx_parallel_dnload_frm(adf_nbuf_t buf)
{
   return __adf_nbuf_get_tx_htt2_frm(buf);
}

/**
 * adf_nbuf_get_dhcp_subtype() - get the subtype
 *              of DHCP packet.
 * @buf: Pointer to DHCP packet buffer
 *
 * This func. returns the subtype of DHCP packet.
 *
 * Return: subtype of the DHCP packet.
 */
static inline enum adf_proto_subtype
adf_nbuf_get_dhcp_subtype(adf_nbuf_t buf)
{
	return __adf_nbuf_data_get_dhcp_subtype(adf_nbuf_data(buf));
}

/**
 * adf_nbuf_data_get_dhcp_subtype() - get the subtype
 *              of DHCP packet.
 * @buf: Pointer to DHCP packet data buffer
 *
 * This func. returns the subtype of DHCP packet.
 *
 * Return: subtype of the DHCP packet.
 */
static inline enum adf_proto_subtype
adf_nbuf_data_get_dhcp_subtype(uint8_t *data)
{
	return __adf_nbuf_data_get_dhcp_subtype(data);
}

/**
 * adf_nbuf_get_eapol_subtype() - get the subtype
 *            of EAPOL packet.
 * @buf: Pointer to EAPOL packet buffer
 *
 * This func. returns the subtype of EAPOL packet.
 *
 * Return: subtype of the EAPOL packet.
 */
static inline enum adf_proto_subtype
adf_nbuf_get_eapol_subtype(adf_nbuf_t buf)
{
	return __adf_nbuf_data_get_eapol_subtype(adf_nbuf_data(buf));
}

/**
 * adf_nbuf_data_get_eapol_subtype() - get the subtype
 *            of EAPOL packet.
 * @data: Pointer to EAPOL packet data buffer
 *
 * This func. returns the subtype of EAPOL packet.
 *
 * Return: subtype of the EAPOL packet.
 */
static inline enum adf_proto_subtype
adf_nbuf_data_get_eapol_subtype(uint8_t *data)
{
	return __adf_nbuf_data_get_eapol_subtype(data);
}

/**
 * adf_nbuf_get_arp_subtype() - get the subtype
 *            of ARP packet.
 * @buf: Pointer to ARP packet buffer
 *
 * This func. returns the subtype of ARP packet.
 *
 * Return: subtype of the ARP packet.
 */
static inline enum adf_proto_subtype
adf_nbuf_get_arp_subtype(adf_nbuf_t buf)
{
	return __adf_nbuf_data_get_arp_subtype(adf_nbuf_data(buf));
}

/**
 * adf_nbuf_data_get_arp_subtype() - get the subtype
 *            of ARP packet.
 * @data: Pointer to ARP packet data buffer
 *
 * This func. returns the subtype of ARP packet.
 *
 * Return: subtype of the ARP packet.
 */
static inline enum adf_proto_subtype
adf_nbuf_data_get_arp_subtype(uint8_t *data)
{
	return __adf_nbuf_data_get_arp_subtype(data);
}

/**
 * adf_nbuf_get_icmp_subtype() - get the subtype
 *            of IPV4 ICMP packet.
 * @buf: Pointer to IPV4 ICMP packet buffer
 *
 * This func. returns the subtype of ICMP packet.
 *
 * Return: subtype of the ICMP packet.
 */
static inline enum adf_proto_subtype
adf_nbuf_get_icmp_subtype(adf_nbuf_t buf)
{
	return __adf_nbuf_data_get_icmp_subtype(adf_nbuf_data(buf));
}

/**
 * adf_nbuf_data_get_icmp_subtype() - get the subtype
 *            of IPV4 ICMP packet.
 * @data: Pointer to IPV4 ICMP packet data buffer
 *
 * This func. returns the subtype of ICMP packet.
 *
 * Return: subtype of the ICMP packet.
 */
static inline enum adf_proto_subtype
adf_nbuf_data_get_icmp_subtype(uint8_t *data)
{
	return __adf_nbuf_data_get_icmp_subtype(data);
}

/**
 * adf_nbuf_get_icmpv6_subtype() - get the subtype
 *            of IPV6 ICMPV6 packet.
 * @buf: Pointer to IPV6 ICMPV6 packet buffer
 *
 * This func. returns the subtype of ICMPV6 packet.
 *
 * Return: subtype of the ICMPV6 packet.
 */
static inline enum adf_proto_subtype
adf_nbuf_get_icmpv6_subtype(adf_nbuf_t buf)
{
	return __adf_nbuf_data_get_icmpv6_subtype(adf_nbuf_data(buf));
}

/**
 * adf_nbuf_data_get_icmpv6_subtype() - get the subtype
 *            of IPV6 ICMPV6 packet.
 * @data: Pointer to IPV6 ICMPV6 packet data buffer
 *
 * This func. returns the subtype of ICMPV6 packet.
 *
 * Return: subtype of the ICMPV6 packet.
 */
static inline enum adf_proto_subtype
adf_nbuf_data_get_icmpv6_subtype(uint8_t *data)
{
	return __adf_nbuf_data_get_icmpv6_subtype(data);
}

/**
 * adf_nbuf_data_get_ipv4_proto() - get the proto type
 *            of IPV4 packet.
 * @data: Pointer to IPV4 packet data buffer
 *
 * This func. returns the proto type of IPV4 packet.
 *
 * Return: proto type of IPV4 packet.
 */
static inline uint8_t
adf_nbuf_data_get_ipv4_proto(uint8_t *data)
{
	return __adf_nbuf_data_get_ipv4_proto(data);
}

/**
 * adf_nbuf_data_get_ipv6_proto() - get the proto type
 *            of IPV6 packet.
 * @data: Pointer to IPV6 packet data buffer
 *
 * This func. returns the proto type of IPV6 packet.
 *
 * Return: proto type of IPV6 packet.
 */
static inline uint8_t
adf_nbuf_data_get_ipv6_proto(uint8_t *data)
{
	return __adf_nbuf_data_get_ipv6_proto(data);
}

/**
 * adf_nbuf_is_dhcp_pkt() - check if it is DHCP packet.
 * @buf: Pointer to DHCP packet buffer
 *
 * This func. checks whether it is a DHCP packet or not.
 *
 * Return: TRUE if it is a DHCP packet
 *         FALSE if not
 */
static inline
bool adf_nbuf_is_dhcp_pkt(adf_nbuf_t buf)
{
	return __adf_nbuf_data_is_dhcp_pkt(adf_nbuf_data(buf));
}

/**
 * adf_nbuf_data_is_dhcp_pkt() - check if it is DHCP packet.
 * @data: Pointer to DHCP packet data buffer
 *
 * This func. checks whether it is a DHCP packet or not.
 *
 * Return: TRUE if it is a DHCP packet
 *         FALSE if not
 */
static inline
bool adf_nbuf_data_is_dhcp_pkt(uint8_t *data)
{
	return __adf_nbuf_data_is_dhcp_pkt(data);
}

/**
 * adf_nbuf_is_eapol_pkt() - check if it is EAPOL packet.
 * @buf: Pointer to EAPOL packet buffer
 *
 * This func. checks whether it is a EAPOL packet or not.
 *
 * Return: TRUE if it is a EAPOL packet
 *         FALSE if not
 */
static inline
bool adf_nbuf_is_eapol_pkt(adf_nbuf_t buf)
{
	return __adf_nbuf_data_is_eapol_pkt(adf_nbuf_data(buf));
}

/**
 * adf_nbuf_data_is_eapol_pkt() - check if it is EAPOL packet.
 * @data: Pointer to EAPOL packet data buffer
 *
 * This func. checks whether it is a EAPOL packet or not.
 *
 * Return: TRUE if it is a EAPOL packet
 *         FALSE if not
 */
static inline
bool adf_nbuf_data_is_eapol_pkt(uint8_t *data)
{
	return __adf_nbuf_data_is_eapol_pkt(data);
}

/**
 * adf_nbuf_is_ipv4_arp_pkt() - check if it is ARP packet.
 * @buf: Pointer to ARP packet buffer
 *
 * This func. checks whether it is a ARP packet or not.
 *
 * Return: TRUE if it is a ARP packet
 *         FALSE if not
 */
static inline
bool adf_nbuf_is_ipv4_arp_pkt(adf_nbuf_t buf)
{
	return __adf_nbuf_data_is_ipv4_arp_pkt(adf_nbuf_data(buf));
}

/**
 * adf_nbuf_data_is_ipv4_arp_pkt() - check if it is ARP packet.
 * @data: Pointer to ARP packet data buffer
 *
 * This func. checks whether it is a ARP packet or not.
 *
 * Return: TRUE if it is a ARP packet
 *         FALSE if not
 */
static inline
bool adf_nbuf_data_is_ipv4_arp_pkt(uint8_t *data)
{
	return __adf_nbuf_data_is_ipv4_arp_pkt(data);
}

/**
 * adf_nbuf_is_ipv4_pkt() - check if it is IPV4 packet.
 * @buf: Pointer to IPV4 packet buffer
 *
 * This func. checks whether it is a IPV4 packet or not.
 *
 * Return: TRUE if it is a IPV4 packet
 *         FALSE if not
 */
static inline
bool adf_nbuf_is_ipv4_pkt(adf_nbuf_t buf)
{
	return __adf_nbuf_data_is_ipv4_pkt(adf_nbuf_data(buf));
}

/**
 * adf_nbuf_data_is_ipv4_pkt() - check if it is IPV4 packet.
 * @data: Pointer to IPV4 packet data buffer
 *
 * This func. checks whether it is a IPV4 packet or not.
 *
 * Return: TRUE if it is a IPV4 packet
 *         FALSE if not
 */
static inline
bool adf_nbuf_data_is_ipv4_pkt(uint8_t *data)
{
	return __adf_nbuf_data_is_ipv4_pkt(data);
}

/**
 * adf_nbuf_data_is_ipv4_mcast_pkt() - check if it is IPV4 multicast packet.
 * @data: Pointer to IPV4 packet data buffer
 *
 * This func. checks whether it is a IPV4 multicast packet or not.
 *
 * Return: TRUE if it is a IPV4 multicast packet
 *         FALSE if not
 */
static inline
bool adf_nbuf_data_is_ipv4_mcast_pkt(uint8_t *data)
{
	return __adf_nbuf_data_is_ipv4_mcast_pkt(data);
}

/**
 * adf_nbuf_data_is_ipv6_mcast_pkt() - check if it is IPV6 multicast packet.
 * @data: Pointer to IPV6 packet data buffer
 *
 * This func. checks whether it is a IPV6 multicast packet or not.
 *
 * Return: TRUE if it is a IPV6 multicast packet
 *         FALSE if not
 */
static inline
bool adf_nbuf_data_is_ipv6_mcast_pkt(uint8_t *data)
{
	return __adf_nbuf_data_is_ipv6_mcast_pkt(data);
}

/**
 * adf_nbuf_is_ipv6_pkt() - check if it is IPV6 packet.
 * @buf: Pointer to IPV6 packet buffer
 *
 * This func. checks whether it is a IPV6 packet or not.
 *
 * Return: TRUE if it is a IPV6 packet
 *         FALSE if not
 */
static inline
bool adf_nbuf_is_ipv6_pkt(adf_nbuf_t buf)
{
	return __adf_nbuf_data_is_ipv6_pkt(adf_nbuf_data(buf));
}

/**
 * adf_nbuf_data_is_ipv6_pkt() - check if it is IPV6 packet.
 * @data: Pointer to IPV6 packet data buffer
 *
 * This func. checks whether it is a IPV6 packet or not.
 *
 * Return: TRUE if it is a IPV6 packet
 *         FALSE if not
 */
static inline
bool adf_nbuf_data_is_ipv6_pkt(uint8_t *data)
{
	return __adf_nbuf_data_is_ipv6_pkt(data);
}

/**
 * adf_nbuf_is_icmp_pkt() - check if it is IPV4 ICMP packet.
 * @buf: Pointer to IPV4 ICMP packet buffer
 *
 * This func. checks whether it is a ICMP packet or not.
 *
 * Return: TRUE if it is a ICMP packet
 *         FALSE if not
 */
static inline
bool adf_nbuf_is_icmp_pkt(adf_nbuf_t buf)
{
	return __adf_nbuf_data_is_icmp_pkt(adf_nbuf_data(buf));
}

/**
 * adf_nbuf_data_is_icmp_pkt() - check if it is IPV4 ICMP packet.
 * @data: Pointer to IPV4 ICMP packet data buffer
 *
 * This func. checks whether it is a ICMP packet or not.
 *
 * Return: TRUE if it is a ICMP packet
 *         FALSE if not
 */
static inline
bool adf_nbuf_data_is_icmp_pkt(uint8_t *data)
{
	return __adf_nbuf_data_is_icmp_pkt(data);
}

/**
 * adf_nbuf_is_icmpv6_pkt() - check if it is IPV6 ICMPV6 packet.
 * @buf: Pointer to IPV6 ICMPV6 packet buffer
 *
 * This func. checks whether it is a ICMPV6 packet or not.
 *
 * Return: TRUE if it is a ICMPV6 packet
 *         FALSE if not
 */
static inline
bool adf_nbuf_is_icmpv6_pkt(adf_nbuf_t buf)
{
	return __adf_nbuf_data_is_icmpv6_pkt(adf_nbuf_data(buf));
}

/**
 * adf_nbuf_data_is_icmpv6_pkt() - check if it is IPV6 ICMPV6 packet.
 * @data: Pointer to IPV6 ICMPV6 packet data buffer
 *
 * This func. checks whether it is a ICMPV6 packet or not.
 *
 * Return: TRUE if it is a ICMPV6 packet
 *         FALSE if not
 */
static inline
bool adf_nbuf_data_is_icmpv6_pkt(uint8_t *data)
{
	return __adf_nbuf_data_is_icmpv6_pkt(data);
}

/**
 * adf_nbuf_is_ipv4_udp_pkt() - check if it is IPV4 UDP packet.
 * @buf: Pointer to IPV4 UDP packet buffer
 *
 * This func. checks whether it is a IPV4 UDP packet or not.
 *
 * Return: TRUE if it is a IPV4 UDP packet
 *         FALSE if not
 */
static inline
bool adf_nbuf_is_ipv4_udp_pkt(adf_nbuf_t buf)
{
	return __adf_nbuf_data_is_ipv4_udp_pkt(adf_nbuf_data(buf));
}

/**
 * adf_nbuf_data_is_ipv4_udp_pkt() - check if it is IPV4 UDP packet.
 * @data: Pointer to IPV4 UDP packet data buffer
 *
 * This func. checks whether it is a IPV4 UDP packet or not.
 *
 * Return: TRUE if it is a IPV4 UDP packet
 *         FALSE if not
 */
static inline
bool adf_nbuf_data_is_ipv4_udp_pkt(uint8_t *data)
{
	return __adf_nbuf_data_is_ipv4_udp_pkt(data);
}

/**
 * adf_nbuf_is_ipv4_tcp_pkt() - check if it is IPV4 TCP packet.
 * @buf: Pointer to IPV4 TCP packet buffer
 *
 * This func. checks whether it is a IPV4 TCP packet or not.
 *
 * Return: TRUE if it is a IPV4 TCP packet
 *         FALSE if not
 */
static inline
bool adf_nbuf_is_ipv4_tcp_pkt(adf_nbuf_t buf)
{
	return __adf_nbuf_data_is_ipv4_tcp_pkt(adf_nbuf_data(buf));
}

/**
 * adf_nbuf_data_is_ipv4_tcp_pkt() - check if it is IPV4 TCP packet.
 * @data: Pointer to IPV4 TCP packet data buffer
 *
 * This func. checks whether it is a IPV4 TCP packet or not.
 *
 * Return: TRUE if it is a IPV4 TCP packet
 *         FALSE if not
 */
static inline
bool adf_nbuf_data_is_ipv4_tcp_pkt(uint8_t *data)
{
	return __adf_nbuf_data_is_ipv4_tcp_pkt(data);
}

/**
 * adf_nbuf_is_ipv6_udp_pkt() - check if it is IPV6 UDP packet.
 * @buf: Pointer to IPV6 UDP packet buffer
 *
 * This func. checks whether it is a IPV6 UDP packet or not.
 *
 * Return: TRUE if it is a IPV6 UDP packet
 *         FALSE if not
 */
static inline
bool adf_nbuf_is_ipv6_udp_pkt(adf_nbuf_t buf)
{
	return __adf_nbuf_data_is_ipv6_udp_pkt(adf_nbuf_data(buf));
}

/**
 * adf_nbuf_data_is_ipv6_udp_pkt() - check if it is IPV6 UDP packet.
 * @data: Pointer to IPV6 UDP packet data buffer
 *
 * This func. checks whether it is a IPV6 UDP packet or not.
 *
 * Return: TRUE if it is a IPV6 UDP packet
 *         FALSE if not
 */
static inline
bool adf_nbuf_data_is_ipv6_udp_pkt(uint8_t *data)
{
	return __adf_nbuf_data_is_ipv6_udp_pkt(data);
}

/**
 * adf_nbuf_is_ipv6_tcp_pkt() - check if it is IPV6 TCP packet.
 * @buf: Pointer to IPV6 TCP packet buffer
 *
 * This func. checks whether it is a IPV6 TCP packet or not.
 *
 * Return: TRUE if it is a IPV6 TCP packet
 *         FALSE if not
 */
static inline
bool adf_nbuf_is_ipv6_tcp_pkt(adf_nbuf_t buf)
{
	return __adf_nbuf_data_is_ipv6_tcp_pkt(adf_nbuf_data(buf));
}

/**
 * adf_nbuf_data_is_ipv6_tcp_pkt() - check if it is IPV6 TCP packet.
 * @data: Pointer to IPV6 TCP packet data buffer
 *
 * This func. checks whether it is a IPV6 TCP packet or not.
 *
 * Return: TRUE if it is a IPV6 TCP packet
 *         FALSE if not
 */
static inline
bool adf_nbuf_data_is_ipv6_tcp_pkt(uint8_t *data)
{
	return __adf_nbuf_data_is_ipv6_tcp_pkt(data);
}

/**
 * adf_nbuf_update_radiotap() - update radiotap at head of nbuf.
 * @rx_status: rx_status containing required info to update radiotap
 * @nbuf: Pointer to nbuf
 * @headroom_sz: Available headroom size
 *
 * Return: radiotap length.
 */
int adf_nbuf_update_radiotap(struct mon_rx_status *rx_status, adf_nbuf_t nbuf,
			     u_int32_t headroom_sz);

/**
 * adf_nbuf_construct_radiotap() - fill in the info into radiotap buf
 *
 * @rtap_buf: pointer to radiotap buffer
 * @tsf: timestamp of packet
 * @rssi_comb: rssi of packet
 *
 * Return: length of rtap_len updated.
 */
uint16_t adf_nbuf_construct_radiotap(
		uint8_t *rtap_buf,
		uint32_t tsf,
		uint32_t rssi_comb);

/**
 * adf_nbuf_update_skb_mark() - update skb->mark.
 * @skb: Pointer to nbuf
 * @mask: the mask to set in skb->mark
 *
 * Return: None
 */
static inline void
adf_nbuf_update_skb_mark(adf_nbuf_t skb, uint32_t mask)
{
	 __adf_nbuf_update_skb_mark(skb, mask);
}

/**
 * adf_nbuf_is_wai() - Check if frame is WAI
 * @skb: Pointer to skb
 *
 * This function checks if the frame is WAPI.
 *
 * Return: true (1) if WAPI
 *
 */
static inline bool adf_nbuf_is_wai_pkt(struct sk_buff *skb)
{
	return __adf_nbuf_is_wai_pkt(skb->data);
}

/**
 * adf_nbuf_is_multicast_pkt() - Check if frame is multicast packet
 * @skb: Pointer to skb
 *
 * This function checks if the frame is multicast packet.
 *
 * Return: true (1) if multicast
 *
 */
static inline bool adf_nbuf_is_multicast_pkt(struct sk_buff *skb)
{
	return __adf_nbuf_is_multicast_pkt(skb->data);
}

/**
 * adf_nbuf_is_bcast_pkt() - Check if frame is broadcast packet
 * @skb: Pointer to skb
 *
 * This function checks if the frame is broadcast packet.
 *
 * Return: true (1) if broadcast
 *
 */
static inline bool adf_nbuf_is_bcast_pkt(struct sk_buff *skb)
{
	return __adf_nbuf_is_bcast_pkt(skb->data);
}




void adf_nbuf_set_state(adf_nbuf_t nbuf, uint8_t current_state);
void adf_nbuf_tx_desc_count_display(void);
void adf_nbuf_tx_desc_count_clear(void);

#endif
