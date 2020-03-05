/** @file  mlan2040coex.h
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
     06/24/2009: initial version
************************************************************************/
#ifndef _COEX_H_
#define _COEX_H_

/** NXP private command identifier */
#define CMD_NXP "MRVL_CMD"

/** IOCTL number */
#define MLAN_ETH_PRIV       (SIOCDEVPRIVATE + 14)

#if (BYTE_ORDER == LITTLE_ENDIAN)
#undef BIG_ENDIAN_SUPPORT
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

/** 64 bits byte swap */
#define swap_byte_64(x) \
  ((t_u64)((t_u64)(((t_u64)(x) & 0x00000000000000ffULL) << 56) | \
         (t_u64)(((t_u64)(x) & 0x000000000000ff00ULL) << 40) | \
         (t_u64)(((t_u64)(x) & 0x0000000000ff0000ULL) << 24) | \
         (t_u64)(((t_u64)(x) & 0x00000000ff000000ULL) <<  8) | \
         (t_u64)(((t_u64)(x) & 0x000000ff00000000ULL) >>  8) | \
         (t_u64)(((t_u64)(x) & 0x0000ff0000000000ULL) >> 24) | \
         (t_u64)(((t_u64)(x) & 0x00ff000000000000ULL) >> 40) | \
         (t_u64)(((t_u64)(x) & 0xff00000000000000ULL) >> 56) ))

/** Convert to correct endian format */
#ifdef 	BIG_ENDIAN_SUPPORT
/** CPU to little-endian convert for 16-bit */
#define 	cpu_to_le16(x)	swap_byte_16(x)
/** CPU to little-endian convert for 32-bit */
#define		cpu_to_le32(x)  swap_byte_32(x)
/** CPU to little-endian convert for 64-bit */
#define     cpu_to_le64(x)  swap_byte_64(x)
/** Little-endian to CPU convert for 16-bit */
#define 	le16_to_cpu(x)	swap_byte_16(x)
/** Little-endian to CPU convert for 32-bit */
#define		le32_to_cpu(x)  swap_byte_32(x)
/** Little-endian to CPU convert for 64-bit */
#define     le64_to_cpu(x)  swap_byte_64(x)
#else
/** Do nothing */
#define		cpu_to_le16(x)	(x)
/** Do nothing */
#define		cpu_to_le32(x)  (x)
/** Do nothing */
#define		cpu_to_le64(x)  (x)
/** Do nothing */
#define 	le16_to_cpu(x)	(x)
/** Do nothing */
#define 	le32_to_cpu(x)	(x)
/** Do nothing */
#define 	le64_to_cpu(x)	(x)
#endif

#ifdef __GNUC__
/** Structure packing begins */
#define PACK_START
/** Structure packeing end */
#define PACK_END  __attribute__ ((packed))
#else
/** Structure packing begins */
#define PACK_START   __packed
/** Structure packeing end */
#define PACK_END
#endif

/** Character, 1 byte */
typedef signed char t_s8;
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

/** Type definition: boolean */
typedef enum { FALSE, TRUE } boolean;

/** Find number of elements */
#define NELEMENTS(x) (sizeof(x)/sizeof(x[0]))
/** Success */
#define MLAN_STATUS_SUCCESS         (0)
/** Failure */
#define MLAN_STATUS_FAILURE         (-1)

/** Enumeration for host-command index */
enum COMMANDS {
	CMD_2040COEX = 1,
};
/** Maximum number of channels that can be sent in a setuserscan ioctl */
#define WLAN_IOCTL_USER_SCAN_CHAN_MAX  50

#ifndef ETH_ALEN
/** MAC address length */
#define ETH_ALEN    6
#endif

/** Netlink protocol number */
#define NETLINK_NXP         (MAX_LINKS - 1)
/** Netlink maximum payload size */
#define NL_MAX_PAYLOAD          1024
/** Netlink multicast group number */
#define NL_MULTICAST_GROUP      RTMGRP_LINK
/** Default wait time in seconds for events */
#define UAP_RECV_WAIT_DEFAULT   10
#ifndef NLMSG_HDRLEN
/** NL message header length */
#define NLMSG_HDRLEN            ((int) NLMSG_ALIGN(sizeof(struct nlmsghdr)))
#endif

/** Event header */
typedef PACK_START struct _event_header {
    /** Event ID */
	t_u32 event_id;
    /** Event data */
	t_u8 event_data[0];
} PACK_END event_header;

/** Event ID length */
#define EVENT_ID_LEN    4

/** Custom events definitions */
/** AP connected event */
#define CUS_EVT_AP_CONNECTED        "EVENT=AP_CONNECTED"

/** Custom event : BW changed */
#define CUS_EVT_BW_CHANGED		    "EVENT=BW_CHANGED"
/** Custom event : OBSS scan parameter */
#define CUS_EVT_OBSS_SCAN_PARAM		"EVENT=OBSS_SCAN_PARAM"

/** Custom events definitions end */

/** Structure defination of chan_intol_t*/
typedef struct _chan_intol_t {
    /** Channel numer */
	t_u8 chan_num;
    /** Flag: Is any 40MHz intolerant AP found in this channel */
	t_u8 is_intol_set;
} chan_intol_t;

/** Private command structure */
#ifdef USERSPACE_32BIT_OVER_KERNEL_64BIT
struct eth_priv_cmd {
    /** Command buffer pointer */
	t_u64 buf;
    /** buffer updated by driver */
	int used_len;
    /** buffer sent by application */
	int total_len;
} __ATTRIB_PACK__;
#else
struct eth_priv_cmd {
    /** Command buffer */
	t_u8 *buf;
    /** Used length */
	int used_len;
    /** Total length */
	int total_len;
};
#endif

/** Legacy APs channel list */
chan_intol_t leg_ap_chan_list[WLAN_IOCTL_USER_SCAN_CHAN_MAX];
/** Total number of channel present in Legacy APs channel list */
t_u8 num_leg_ap_chan;
int get_region_code(int *reg_code);
int process_host_cmd(int cmd, t_u8 *chan_list, t_u8 chan_num, t_u8 reg_class,
		     t_u8 is_intol_ap_present);
int is_intolerant_sta(int *intol);

#endif /* _COEX_H_ */
