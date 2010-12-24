/*
 * drivers/net/fec_1588.h
 *
 * Copyright (C) 2010 Freescale Semiconductor, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 *
 */

#ifndef FEC_1588_H
#define FEC_1588_H

#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/circ_buf.h>

#define FALSE			0
#define TRUE			1

/* FEC 1588 register bits */
#define FEC_T_CTRL_SLAVE		0x00002000
#define FEC_T_CTRL_CAPTURE		0x00000800
#define FEC_T_CTRL_RESTART		0x00000200
#define FEC_T_CTRL_PERIOD_RST		0x00000030
#define FEC_T_CTRL_ENABLE		0x00000001

#define FEC_T_INC_MASK			0x0000007f
#define FEC_T_INC_OFFSET		0
#define FEC_T_INC_CORR_MASK		0x00007f00
#define FEC_T_INC_CORR_OFFSET		8

#define FEC_T_INC_40MHZ			25
#define FEC_ATIME_40MHZ			40000000

#define FEC_T_PERIOD_ONE_SEC		0x3B9ACA00

/* IEEE 1588 definition */
#define FEC_ECNTRL_TS_EN	0x10
#define PTP_MAJOR		232	/*the temporary major number
						 *used by PTP driver, the major
						 *number 232~239 is unassigned*/

#define DEFAULT_PTP_RX_BUF_SZ		2048
#define DEFAULT_PTP_TX_BUF_SZ		16
#define PTP_MSG_SYNC			0x0
#define PTP_MSG_DEL_REQ			0x1
#define PTP_MSG_P_DEL_REQ		0x2
#define PTP_MSG_P_DEL_RESP		0x3
#define PTP_MSG_DEL_RESP		0x4
#define PTP_MSG_ALL_OTHER		0x5

#define PTP_GET_TX_TIMESTAMP		0x1
#define PTP_GET_RX_TIMESTAMP		0x9
#define PTP_SET_RTC_TIME		0x3
#define PTP_SET_COMPENSATION		0x4
#define PTP_GET_CURRENT_TIME		0x5
#define PTP_FLUSH_TIMESTAMP		0x6
#define PTP_ADJ_ADDEND			0x7
#define PTP_GET_ORIG_COMP		0x8
#define PTP_GET_ADDEND			0xB
#define PTP_GET_RX_TIMESTAMP_PDELAY_REQ		0xC
#define PTP_GET_RX_TIMESTAMP_PDELAY_RESP	0xD

#define FEC_PTP_DOMAIN_DLFT		0xe0000181
#define FEC_PTP_IP_OFFS			0xE
#define FEC_PTP_UDP_OFFS		0x22
#define FEC_PTP_MSG_TYPE_OFFS		0x2A
#define FEC_PTP_SPORT_ID_OFFS		0x3E
#define FEC_PTP_SEQ_ID_OFFS		0x48
#define FEC_PTP_CTRL_OFFS		0x4A
#define FEC_PACKET_TYPE_UDP		0x11

#define FEC_PTP_ORIG_COMP		0x15555555

/* PTP standard time representation structure */
struct ptp_time{
	u64 sec;	/* seconds */
	u32 nsec;	/* nanoseconds */
};

/* Structure for PTP Time Stamp */
struct fec_ptp_data_t {
	u8		spid[10];
	int		key;
	struct ptp_time	ts_time;
};

/* interface for PTP driver command GET_TX_TIME */
struct ptp_ts_data {
	/* PTP version */
	u8 version;
	/* PTP source port ID */
	u8 spid[10];
	/* PTP sequence ID */
	u16 seq_id;
	/* PTP message type */
	u8 message_type;
	/* PTP timestamp */
	struct ptp_time ts;
};

/* interface for PTP driver command SET_RTC_TIME/GET_CURRENT_TIME */
struct ptp_rtc_time {
	struct ptp_time rtc_time;
};

/* interface for PTP driver command SET_COMPENSATION */
struct ptp_set_comp {
	u32 drift;
	bool o_ops;
	u32 freq_compensation;
};

/* interface for PTP driver command GET_ORIG_COMP */
struct ptp_get_comp {
	/* the initial compensation value */
	u32 dw_origcomp;
	/* the minimum compensation value */
	u32 dw_mincomp;
	/*the max compensation value*/
	u32 dw_maxcomp;
	/*the min drift applying min compensation value in ppm*/
	u32 dw_mindrift;
	/*the max drift applying max compensation value in ppm*/
	u32 dw_maxdrift;
};

struct ptp_time_correct {
	u32 corr_period;
	u32 corr_inc;
};

/* PTP message version */
#define PTP_1588_MSG_VER_1	1
#define PTP_1588_MSG_VER_2	2

#define BD_ENET_TX_TS		0x20000000
#define BD_ENET_TX_BDU		0x80000000

struct fec_ptp_private {
	void __iomem *hwp;
	int	dev_id;

	struct	circ_buf rx_time_sync;
	struct	circ_buf rx_time_del_req;
	struct	circ_buf rx_time_pdel_req;
	struct	circ_buf rx_time_pdel_resp;
	struct	circ_buf tx_time_sync;
	struct	circ_buf tx_time_del_req;
	struct	circ_buf tx_time_pdel_req;
	struct	circ_buf tx_time_pdel_resp;
	spinlock_t ptp_lock;
	spinlock_t cnt_lock;

	u64	prtc;
	u8	ptp_active;
	u8	ptp_slave;
	struct circ_buf	txstamp;
};

#ifdef CONFIG_FEC_1588
static inline int fec_ptp_malloc_priv(struct fec_ptp_private **priv)
{
	*priv = kzalloc(sizeof(struct fec_ptp_private), GFP_KERNEL);
	return 1;
}
extern int fec_ptp_init(struct fec_ptp_private *priv, int id);
extern void fec_ptp_cleanup(struct fec_ptp_private *priv);
extern int fec_ptp_start(struct fec_ptp_private *priv);
extern void fec_ptp_stop(struct fec_ptp_private *priv);
extern int fec_ptp_do_txstamp(struct sk_buff *skb);
extern void fec_ptp_store_txstamp(struct fec_ptp_private *priv,
				  struct sk_buff *skb,
				  struct bufdesc *bdp);
extern void fec_ptp_store_rxstamp(struct fec_ptp_private *priv,
				  struct sk_buff *skb,
				  struct bufdesc *bdp);
#else
static inline int fec_ptp_malloc_priv(struct fec_ptp_private **priv)
{
	return 0;
}
static inline int fec_ptp_init(struct fec_ptp_private *priv, int id)
{
	return 1;
}
static inline void fec_ptp_cleanup(struct fec_ptp_private *priv) { }
static inline int fec_ptp_start(struct fec_ptp_private *priv)
{
	return 1;
}
static inline void fec_ptp_stop(struct fec_ptp_private *priv) {}
static inline int fec_ptp_do_txstamp(struct sk_buff *skb)
{
	return 0;
}
static inline void fec_ptp_store_txstamp(struct fec_ptp_private *priv,
					 struct sk_buff *skb,
					 struct bufdesc *bdp) {}
static inline void fec_ptp_store_rxstamp(struct fec_ptp_private *priv,
					 struct sk_buff *skb,
					 struct bufdesc *bdp) {}
#endif /* 1588 */

#endif
