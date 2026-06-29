/* SPDX-License-Identifier: (GPL-2.0+ OR BSD-3-Clause) */
/*
 * Copyright 2025 NXP
 */

#ifndef __ENETC_MAILBOX_H
#define __ENETC_MAILBOX_H

#include <linux/crc-itu-t.h>

#define ENETC_CRC_INIT				0xffff
#define ENETC_MSG_ALIGN				32

#define ENETC_MSG_EXT_BODY_LEN(l)		((l) / ENETC_MSG_ALIGN - 1)
#define ENETC_MSG_SIZE(l)			(((l) + 1) * ENETC_MSG_ALIGN)

/* Common Class ID for PSI-TO-VSI and VSI-TO-PSI messages */
#define ENETC_MSG_CLASS_ID_MAC_FILTER		0x20
#define ENETC_MSG_CLASS_ID_VLAN_FILTER		0x21
#define ENETC_MSG_CLASS_ID_LINK_STATUS		0x80
#define ENETC_MSG_CLASS_ID_LINK_SPEED		0x81
#define ENETC_MSG_CLASS_ID_IP_REVISION		0xf0

/* Class ID for PSI-TO-VSI messages */
#define ENETC_MSG_CLASS_ID_CMD_SUCCESS		0x1
#define ENETC_MSG_CLASS_ID_PERMISSION_DENY	0x2
#define ENETC_MSG_CLASS_ID_CMD_NOT_SUPPORT	0x3
#define ENETC_MSG_CLASS_ID_PSI_BUSY		0x4
#define ENETC_MSG_CLASS_ID_CRC_ERROR		0x5
#define ENETC_MSG_CLASS_ID_PROTO_NOT_SUPPORT	0x6
#define ENETC_MSG_CLASS_ID_INVALID_MSG_LEN	0x7
#define ENETC_MSG_CLASS_ID_CMD_TIMEOUT		0x8
#define ENETC_MSG_CLASS_ID_CMD_DEFERED		0xf

/* Class-specific error return codes for MAC filter */
#define ENETC_PF_RC_MAC_FILTER_INVALID_MAC	0x0
#define ENETC_PF_RC_MAC_FILTER_DUPLICATE_MAC	0x1
#define ENETC_PF_RC_MAC_FILTER_MAC_NOT_FOUND	0x2
#define ENETC_PF_RC_MAC_FILTER_NO_RESOURCE	0x3

/* Class-specific error return codes for IP revision */
#define ENETC_PF_RC_IP_REVISION_INVALID		0xff

/* Class-specific notification codes for link status */
#define ENETC_PF_NC_LINK_STATUS_UP		0x0
#define ENETC_PF_NC_LINK_STATUS_DOWN		0x1

/* Class-specific error return codes for VLAN filter */
#define ENETC_PF_RC_VLAN_FILTER_INVALID_VLAN	0x0
#define ENETC_PF_RC_VLAN_FILTER_DUPLICATE_VLAN	0x1
#define ENETC_PF_RC_VLAN_FILTER_VLAN_NOT_FOUND	0x2
#define ENETC_PF_RC_VLAN_FILTER_NO_RESOURCE	0x3

#define ENETC_MAC_FILTER_TYPE_UC		BIT(0)
#define ENETC_MAC_FILTER_TYPE_MC		BIT(1)
#define ENETC_MAC_FILTER_TYPE_ALL		(ENETC_MAC_FILTER_TYPE_UC | \
						 ENETC_MAC_FILTER_TYPE_MC)

#define ENETC_MAC_PROMISC_MODE_DISABLE		0
#define ENETC_MAC_PROMISC_MODE_ENABLE		1
#define ENETC_MAC_FILTER_FLUSH			1
#define ENETC_MAC_HASH_TABLE_SIZE_64		0

#define ENETC_VLAN_HASH_TABLE_SIZE_64		0
#define ENETC_VLAN_PROMISC_MODE_DISABLE		0
#define ENETC_VLAN_PROMISC_MODE_ENABLE		1

enum enetc_msg_mac_filter_cmd_id {
	ENETC_MSG_SET_PRIMARY_MAC,
	ENETC_MSG_ADD_EXACT_MAC_ENTRIES,
	ENETC_MSG_DEL_EXACT_MAC_ENTRIES,
	ENETC_MSG_SET_MAC_HASH_TABLE,
	ENETC_MSG_FLUSH_MAC_ENTRIES,
	ENETC_MSG_SET_MAC_PROMISC_MODE,
};

enum enetc_msg_ip_revision_cmd_id {
	ENETC_MSG_GET_IP_MN = 1,
};

enum enetc_msg_link_status_cmd_id {
	ENETC_MSG_GET_CURRENT_LINK_STATUS,
	ENETC_MSG_REGISTER_LINK_CHANGE_NOTIFY,
	ENETC_MSG_UNREGISTER_LINK_CHANGE_NOTIFY,
};

enum enetc_msg_link_speed_cmd_id {
	ENETC_MSG_GET_CURRENT_LINK_SPEED,
	ENETC_MSG_REGISTER_SPEED_CHANGE_NOTIFY,
	ENETC_MSG_UNREGISTER_SPEED_CHANGE_NOTIFY,
};

enum enetc_msg_link_speed_val {
	ENETC_MSG_SPEED_UNKNOWN,
	ENETC_MSG_SPEED_10M_HD,
	ENETC_MSG_SPEED_10M_FD,
	ENETC_MSG_SPEED_100M_HD,
	ENETC_MSG_SPEED_100M_FD,
	ENETC_MSG_SPEED_1000M,
	ENETC_MSG_SPEED_2500M,
	ENETC_MSG_SPEED_5G,
	/* Do not add enumeration values for any speed greater than
	 * 5Gbps. For any speed greater than 5Gbps, its speed class
	 * code should follow the formula below.
	 *
	 * SPEED = (link_speed - 5000) / 1000 + ENETC_MSG_SPEED_5G
	 *
	 * The unit of link_speed should be Mbps, the max SPEED
	 * should <= ENETC_MSG_SPEED_MAX.
	 */
	ENETC_MSG_SPEED_MAX = 0xff,
};

enum enetc_msg_vlan_filter_cmd_id {
	ENETC_MSG_ADD_EXACT_VLAN_ENTRIES,
	ENETC_MSG_DEL_EXACT_VLAN_ENTRIES,
	ENETC_MSG_SET_VLAN_HASH_TABLE,
	ENETC_MSG_FLUSH_VLAN_ENTRIES,
	ENETC_MSG_SET_VLAN_PROMISC_MODE,
};

struct enetc_msg_swbd {
	void *vaddr;
	dma_addr_t dma;
	int size;
	/* save return code from PSI for 'get' messages */
	u8 class_code;
};

/* The format of PSI-TO-VSI message, only a 16-bits code */
union enetc_pf_msg {
	struct {
		union {
			struct {
				u8 cookie:4;
				u8 class_code:4;
			};
			/* some messages class_code is 8-bit without cookie */
			u8 class_code_u8;
		};
		u8 class_id;
	};
	u16 code;
};

/* The formats of VSI-TO-PSI messages */
struct enetc_msg_header {
	__be16 crc16;
	u8 class_id;
	u8 cmd_id;
	u8 proto_ver;
	u8 len;
	u8 resv0;
	u8 cookie:4;
	u8 resv1:4;
	u8 resv2[8];
};

struct enetc_mac_addr {
	u8 addr[ETH_ALEN];
};

/* message format of class_id 0x20, cmd_id: 0x0
 * cmd_id 0: set primary MAC
 */
struct enetc_msg_mac_exact_filter {
	struct enetc_msg_header hdr;
	u8 mac_cnt;
	u8 resv[3];
	struct enetc_mac_addr mac[];
};

/* message format of class_id 0x20, cmd_id: 0x3,
 * set MAC hash table
 */
struct enetc_msg_mac_hash_filter {
	struct enetc_msg_header hdr;
	u8 size:6;
	u8 type:2;
	u8 resv[3];
	u32 hash_tbl[];
};

/* message format of class_id 0x20, cmd_id: 0x5,
 * set MAC promiscuous mode
 */
struct enetc_msg_mac_promsic_mode {
	struct enetc_msg_header hdr;
	u8 flush_macs:1;
	u8 promisc_mode:1;
	u8 resv:4;
	u8 type:2;
};

/* message format of class_id 0x21, cmd_id: 0x2,
 * set VLAN hash table
 */
struct enetc_msg_vlan_hash_filter {
	struct enetc_msg_header hdr;
	u8 size;
	u8 resv[3];
	u32 hash_tbl[];
};

/* message format of class_id 0x21, cmd_id: 0x4,
 * set VLAN promiscuous mode
 */
struct enetc_msg_vlan_promsic_mode {
	struct enetc_msg_header hdr;
	u8 flush_vlans:1;
	u8 promisc_mode:1;
	u8 resv:6;
};

/* The generic message format applies to the following messages:
 * Get IP revision message, class_id: 0xf0
 * cmd_id 1: get IP minor revision
 *
 * Link status message, class id: 0x80
 * cmd_id 0x0: get the current link speed
 * cmd_id 0x1: register to link speed change notification
 * cmd_id 0x2: unregister from link speed change notification
 *
 * Link speed message, class_id: 0x81
 * cmd_id 0x0: get the current link speed
 * cmd_id 0x1: register to link speed change notification
 * cmd_id 0x2: unregister from link speed change notification
 */
struct enetc_msg_generic {
	struct enetc_msg_header hdr;
};

#endif
