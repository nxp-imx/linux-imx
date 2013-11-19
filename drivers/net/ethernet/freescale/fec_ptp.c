/*
 * Fast Ethernet Controller (ENET) PTP driver for MX6x.
 *
 * Copyright (C) 2012-2013 Freescale Semiconductor, Inc.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin St - Fifth Floor, Boston, MA 02110-1301 USA.
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/ptrace.h>
#include <linux/errno.h>
#include <linux/ioport.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/pci.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/skbuff.h>
#include <linux/spinlock.h>
#include <linux/workqueue.h>
#include <linux/bitops.h>
#include <linux/io.h>
#include <linux/vmalloc.h>
#include <linux/ip.h>
#include <linux/udp.h>
#include <linux/irq.h>
#include <linux/clk.h>
#include <linux/platform_device.h>
#include <linux/phy.h>
#include <linux/fec.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/of_net.h>

#include "fec.h"


/* FEC 1588 register bits */
#define FEC_T_CTRL_SLAVE                0x00002000
#define FEC_T_CTRL_CAPTURE              0x00000800
#define FEC_T_CTRL_RESTART              0x00000200
#define FEC_T_CTRL_PERIOD_RST           0x00000030
#define FEC_T_CTRL_PERIOD_EN		0x00000010
#define FEC_T_CTRL_ENABLE               0x00000001

#define FEC_T_INC_MASK                  0x0000007f
#define FEC_T_INC_OFFSET                0
#define FEC_T_INC_CORR_MASK             0x00007f00
#define FEC_T_INC_CORR_OFFSET           8

#define FEC_ATIME_CTRL		0x400
#define FEC_ATIME		0x404
#define FEC_ATIME_EVT_OFFSET	0x408
#define FEC_ATIME_EVT_PERIOD	0x40c
#define FEC_ATIME_CORR		0x410
#define FEC_ATIME_INC		0x414
#define FEC_TS_TIMESTAMP	0x418

#define FEC_CC_MULT	(1 << 31)

/* Alloc the ring resource */
static int fec_ptp_init_circ(struct fec_ptp_circular *buf, int size)
{
	buf->data_buf = (struct fec_ptp_ts_data *)
		vmalloc(size * sizeof(struct fec_ptp_ts_data));

	if (!buf->data_buf)
		return 1;
	buf->front = 0;
	buf->end = 0;
	buf->size = size;
	return 0;
}

static inline int fec_ptp_calc_index(int size, int curr_index, int offset)
{
	return (curr_index + offset) % size;
}

static int fec_ptp_is_empty(struct fec_ptp_circular *buf)
{
	return (buf->front == buf->end);
}

static int fec_ptp_nelems(struct fec_ptp_circular *buf)
{
	const int front = buf->front;
	const int end = buf->end;
	const int size = buf->size;
	int n_items;

	if (end > front)
		n_items = end - front;
	else if (end < front)
		n_items = size - (front - end);
	else
		n_items = 0;

	return n_items;
}

static int fec_ptp_is_full(struct fec_ptp_circular *buf)
{
	if (fec_ptp_nelems(buf) == (buf->size - 1))
		return 1;
	else
		return 0;
}

static int fec_ptp_insert(struct fec_ptp_circular *ptp_buf,
			  struct fec_ptp_ts_data *data)
{
	struct fec_ptp_ts_data *tmp;

	if (fec_ptp_is_full(ptp_buf))
		ptp_buf->end = fec_ptp_calc_index(ptp_buf->size,
						ptp_buf->end, 1);

	tmp = (ptp_buf->data_buf + ptp_buf->end);
	memcpy(tmp, data, sizeof(struct fec_ptp_ts_data));
	ptp_buf->end = fec_ptp_calc_index(ptp_buf->size, ptp_buf->end, 1);

	return 0;
}

static int fec_ptp_find_and_remove(struct fec_ptp_circular *ptp_buf,
			struct fec_ptp_ident *ident, struct ptp_time *ts)
{
	int i;
	int size = ptp_buf->size, end = ptp_buf->end;
	struct fec_ptp_ident *tmp_ident;

	if (fec_ptp_is_empty(ptp_buf))
		return 1;

	i = ptp_buf->front;
	while (i != end) {
		tmp_ident = &(ptp_buf->data_buf + i)->ident;
		if (tmp_ident->version == ident->version) {
			if (tmp_ident->message_type == ident->message_type) {
				if ((tmp_ident->netw_prot == ident->netw_prot)
				|| (ident->netw_prot ==
					FEC_PTP_PROT_DONTCARE)) {
					if (tmp_ident->seq_id ==
							ident->seq_id) {
						int ret =
						memcmp(tmp_ident->spid,
							ident->spid,
							PTP_SOURCE_PORT_LENGTH);
						if (0 == ret)
							break;
					}
				}
			}
		}
		/* get next */
		i = fec_ptp_calc_index(size, i, 1);
	}

	/* not found ? */
	if (i == end) {
		/* buffer full ? */
		if (fec_ptp_is_full(ptp_buf))
			/* drop one in front */
			ptp_buf->front =
			fec_ptp_calc_index(size, ptp_buf->front, 1);

		return 1;
	}
	*ts = (ptp_buf->data_buf + i)->ts;
	ptp_buf->front = fec_ptp_calc_index(size, ptp_buf->front, 1);

	return 0;
}

/* 1588 Module intialization */
void fec_ptp_start(struct net_device *ndev)
{
	struct fec_enet_private *fep = netdev_priv(ndev);
	unsigned long flags;
	int inc;

	inc = FEC_T_PERIOD_ONE_SEC / clk_get_rate(fep->clk_ptp);

	spin_lock_irqsave(&fep->tmreg_lock, flags);
	/* Select 1588 Timer source and enable module for starting Tmr Clock */
	writel(FEC_T_CTRL_RESTART, fep->hwp + FEC_ATIME_CTRL);
	writel(inc << FEC_T_INC_OFFSET,
			fep->hwp + FEC_ATIME_INC);
	writel(FEC_T_PERIOD_ONE_SEC, fep->hwp + FEC_ATIME_EVT_PERIOD);
	/* start counter */
	writel(FEC_T_CTRL_PERIOD_RST | FEC_T_CTRL_ENABLE,
			fep->hwp + FEC_ATIME_CTRL);
	spin_unlock_irqrestore(&fep->tmreg_lock, flags);
}

/* Cleanup routine for 1588 module.
 * When PTP is disabled this routing is called */
void fec_ptp_stop(struct net_device *ndev)
{
	struct fec_enet_private *priv = netdev_priv(ndev);

	writel(0, priv->hwp + FEC_ATIME_CTRL);
	writel(FEC_T_CTRL_RESTART, priv->hwp + FEC_ATIME_CTRL);
}

static void fec_get_curr_cnt(struct fec_enet_private *priv,
			struct ptp_rtc_time *curr_time)
{
	u32 tempval, old_sec;
	u32 timeout_event, timeout_ts = 0;

	do {
		old_sec = priv->prtc;
		timeout_event = 0;

		tempval = readl(priv->hwp + FEC_ATIME_CTRL);
		tempval |= FEC_T_CTRL_CAPTURE;
		writel(tempval, priv->hwp + FEC_ATIME_CTRL);

		curr_time->rtc_time.nsec = readl(priv->hwp + FEC_ATIME);

		while (readl(priv->hwp + FEC_IEVENT) & FEC_ENET_TS_TIMER) {
			timeout_event++;
			udelay(20);

			if (timeout_event >= FEC_PTP_TIMEOUT_EVENT)
				break;
		}

		curr_time->rtc_time.sec = priv->prtc;
		timeout_ts++;

		if (timeout_event >= FEC_PTP_TIMEOUT_EVENT)
			pr_err("timeout: TS TIMER event\n");

	} while (old_sec != curr_time->rtc_time.sec &&
		 timeout_ts < FEC_PTP_TIMEOUT_TS);

	if (timeout_ts >= FEC_PTP_TIMEOUT_TS)
		pr_err("timeout: current timestamp unmatched\n");
}

/* Set the 1588 timer counter registers */
static void fec_set_1588cnt(struct fec_enet_private *priv,
			struct ptp_rtc_time *fec_time)
{
	u32 tempval;
	unsigned long flags;

	spin_lock_irqsave(&priv->tmreg_lock, flags);
	priv->prtc = fec_time->rtc_time.sec;

	tempval = fec_time->rtc_time.nsec;
	writel(tempval, priv->hwp + FEC_ATIME);
	spin_unlock_irqrestore(&priv->tmreg_lock, flags);
}

/**
 * Parse packets if they are PTP.
 * The PTP header can be found in an IPv4, IPv6 or in an IEEE802.3
 * ethernet frame. The function returns the position of the PTP packet
 * or NULL, if no PTP found
 */
u8 *fec_ptp_parse_packet(struct sk_buff *skb, u16 *eth_type)
{
	u8 *position = skb->data + ETH_ALEN + ETH_ALEN;
	u8 *ptp_loc = NULL;

	*eth_type = *((u16 *)position);
	/* Check if outer vlan tag is here */
	if (ntohs(*eth_type) == ETH_P_8021Q) {
		position += FEC_VLAN_TAG_LEN;
		*eth_type = *((u16 *)position);
	}

	/* set position after ethertype */
	position += FEC_ETHTYPE_LEN;
	if (ETH_P_1588 == ntohs(*eth_type)) {
		ptp_loc = position;
		/* IEEE1588 event message which needs timestamping */
		if ((ptp_loc[0] & 0xF) <= 3) {
			if (skb->len >=
			((ptp_loc - skb->data) + PTP_HEADER_SZE))
				return ptp_loc;
		}
	} else if (ETH_P_IP == ntohs(*eth_type)) {
		u8 *ip_header, *prot, *udp_header;
		u8 ip_version, ip_hlen;
		ip_header = position;
		ip_version = ip_header[0] >> 4; /* correct IP version? */
		if (0x04 == ip_version) { /* IPv4 */
			prot = ip_header + 9; /* protocol */
			if (FEC_PACKET_TYPE_UDP == *prot) {
				u16 udp_dstPort;
				/* retrieve the size of the ip-header
				 * with the first byte of the ip-header:
				 * version ( 4 bits) + Internet header
				 * length (4 bits)
				 */
				ip_hlen   = (*ip_header & 0xf) * 4;
				udp_header = ip_header + ip_hlen;
				udp_dstPort = *((u16 *)(udp_header + 2));
				/* check the destination port address
				 * ( 319 (0x013F) = PTP event port )
				 */
				if (ntohs(udp_dstPort) == PTP_EVENT_PORT) {
					ptp_loc = udp_header + 8;
					/* long enough ? */
					if (skb->len >= ((ptp_loc - skb->data)
							+ PTP_HEADER_SZE))
						return ptp_loc;
				}
			}
		}
	} else if (ETH_P_IPV6 == ntohs(*eth_type)) {
		u8 *ip_header, *udp_header, *prot;
		u8 ip_version;
		ip_header = position;
		ip_version = ip_header[0] >> 4;
		if (0x06 == ip_version) {
			prot = ip_header + 6;
			if (FEC_PACKET_TYPE_UDP == *prot) {
				u16 udp_dstPort;
				udp_header = ip_header + 40;
				udp_dstPort = *((u16 *)(udp_header + 2));
				/* check the destination port address
				 * ( 319 (0x013F) = PTP event port )
				 */
				if (ntohs(udp_dstPort) == PTP_EVENT_PORT) {
					ptp_loc = udp_header + 8;
					/* long enough ? */
					if (skb->len >= ((ptp_loc - skb->data)
							+ PTP_HEADER_SZE))
						return ptp_loc;
				}
			}
		}
	}

	return NULL; /* no PTP frame */
}

/* Set the BD to ptp */
int fec_ptp_do_txstamp(struct sk_buff *skb)
{
	u8 *ptp_loc;
	u16 eth_type;

	ptp_loc = fec_ptp_parse_packet(skb, &eth_type);
	if (ptp_loc != NULL)
		return 1;

	return 0;
}

void fec_ptp_store_txstamp(struct fec_enet_private *priv,
			   struct sk_buff *skb,
			   struct bufdesc *bdp)
{
	struct fec_ptp_ts_data tmp_tx_time;
	struct bufdesc_ex *bdp_ex = NULL;
	struct ptp_rtc_time curr_time;
	u8 *ptp_loc;
	u16 eth_type;

	bdp_ex = container_of(bdp, struct bufdesc_ex, desc);
	ptp_loc = fec_ptp_parse_packet(skb, &eth_type);
	if (ptp_loc != NULL) {
		/* store identification data */
		switch (ntohs(eth_type)) {
		case ETH_P_IP:
			tmp_tx_time.ident.netw_prot = FEC_PTP_PROT_IPV4;
			break;
		case ETH_P_IPV6:
			tmp_tx_time.ident.netw_prot = FEC_PTP_PROT_IPV6;
			break;
		case ETH_P_1588:
			tmp_tx_time.ident.netw_prot = FEC_PTP_PROT_802_3;
			break;
		default:
			return;
		}
		tmp_tx_time.ident.version = (*(ptp_loc + 1)) & 0X0F;
		tmp_tx_time.ident.message_type = (*(ptp_loc)) & 0x0F;
		tmp_tx_time.ident.seq_id =
			ntohs(*((u16 *)(ptp_loc + PTP_HEADER_SEQ_OFFS)));
		memcpy(tmp_tx_time.ident.spid, &ptp_loc[PTP_SPID_OFFS],
						PTP_SOURCE_PORT_LENGTH);
		/* store tx timestamp */
		fec_get_curr_cnt(priv, &curr_time);
		if (curr_time.rtc_time.nsec < bdp_ex->ts)
			tmp_tx_time.ts.sec = curr_time.rtc_time.sec - 1;
		else
			tmp_tx_time.ts.sec = curr_time.rtc_time.sec;
		tmp_tx_time.ts.nsec = bdp_ex->ts;
		/* insert timestamp in circular buffer */
		fec_ptp_insert(&(priv->tx_timestamps), &tmp_tx_time);
	}
}

void fec_ptp_store_rxstamp(struct fec_enet_private *priv,
			   struct sk_buff *skb,
			   struct bufdesc *bdp)
{
	struct fec_ptp_ts_data tmp_rx_time;
	struct bufdesc_ex *bdp_ex = NULL;
	struct ptp_rtc_time curr_time;
	u8 *ptp_loc;
	u16 eth_type;

	bdp_ex = container_of(bdp, struct bufdesc_ex, desc);
	ptp_loc = fec_ptp_parse_packet(skb, &eth_type);
	if (ptp_loc != NULL) {
		/* store identification data */
		tmp_rx_time.ident.version = (*(ptp_loc + 1)) & 0X0F;
		tmp_rx_time.ident.message_type = (*(ptp_loc)) & 0x0F;
		switch (ntohs(eth_type)) {
		case ETH_P_IP:
			tmp_rx_time.ident.netw_prot = FEC_PTP_PROT_IPV4;
			break;
		case ETH_P_IPV6:
			tmp_rx_time.ident.netw_prot = FEC_PTP_PROT_IPV6;
			break;
		case ETH_P_1588:
			tmp_rx_time.ident.netw_prot = FEC_PTP_PROT_802_3;
			break;
		default:
			return;
		}
		tmp_rx_time.ident.seq_id =
			ntohs(*((u16 *)(ptp_loc + PTP_HEADER_SEQ_OFFS)));
		memcpy(tmp_rx_time.ident.spid, &ptp_loc[PTP_SPID_OFFS],
						PTP_SOURCE_PORT_LENGTH);
		/* store rx timestamp */
		fec_get_curr_cnt(priv, &curr_time);
		if (curr_time.rtc_time.nsec < bdp_ex->ts)
			tmp_rx_time.ts.sec = curr_time.rtc_time.sec - 1;
		else
			tmp_rx_time.ts.sec = curr_time.rtc_time.sec;
		tmp_rx_time.ts.nsec = bdp_ex->ts;

		/* insert timestamp in circular buffer */
		fec_ptp_insert(&(priv->rx_timestamps), &tmp_rx_time);
	}
}


static void fec_handle_ptpdrift(struct fec_enet_private *priv,
	struct ptp_set_comp *comp, struct ptp_time_correct *ptc)
{
	u32 ndrift;
	u32 i, adj_inc, adj_period;
	u32 tmp_current, tmp_winner;
	u32 ptp_ts_clk, ptp_inc;

	ptp_ts_clk = clk_get_rate(priv->clk_ptp);
	ptp_inc = FEC_T_PERIOD_ONE_SEC / ptp_ts_clk;

	ndrift = comp->drift;

	if (ndrift == 0) {
		ptc->corr_inc = 0;
		ptc->corr_period = 0;
		return;
	} else if (ndrift >= ptp_ts_clk) {
		ptc->corr_inc = (u32)(ndrift / ptp_ts_clk);
		ptc->corr_period = 1;
		return;
	} else {
		tmp_winner = 0xFFFFFFFF;
		adj_inc = 1;

		if (ndrift > (ptp_ts_clk / ptp_inc)) {
			adj_inc = ptp_inc / FEC_PTP_SPINNER_2;
		} else if (ndrift > (ptp_ts_clk /
			(ptp_inc * FEC_PTP_SPINNER_4))) {
			adj_inc = ptp_inc / FEC_PTP_SPINNER_4;
			adj_period = FEC_PTP_SPINNER_2;
		} else {
			adj_inc = FEC_PTP_SPINNER_4;
			adj_period = FEC_PTP_SPINNER_4;
		}

		for (i = 1; i < adj_inc; i++) {
			tmp_current = (ptp_ts_clk * i) % ndrift;
			if (tmp_current == 0) {
				ptc->corr_inc = i;
				ptc->corr_period = (u32)((ptp_ts_clk *
						adj_period * i)	/ ndrift);
				break;
			} else if (tmp_current < tmp_winner) {
				ptc->corr_inc = i;
				ptc->corr_period = (u32)((ptp_ts_clk *
						adj_period * i)	/ ndrift);
				tmp_winner = tmp_current;
			}
		}
	}
}

static void fec_set_drift(struct fec_enet_private *priv,
			  struct ptp_set_comp *comp)
{
	struct ptp_time_correct	tc;
	u32 tmp, corr_ns;
	u32 ptp_inc;

	memset(&tc, 0, sizeof(struct ptp_time_correct));
	fec_handle_ptpdrift(priv, comp, &tc);
	if (tc.corr_inc == 0)
		return;

	ptp_inc = FEC_T_PERIOD_ONE_SEC / clk_get_rate(priv->clk_ptp);
	if (comp->o_ops == TRUE)
		corr_ns = ptp_inc + tc.corr_inc;
	else
		corr_ns = ptp_inc - tc.corr_inc;

	tmp = readl(priv->hwp + FEC_ATIME_INC) & FEC_T_INC_MASK;
	tmp |= corr_ns << FEC_T_INC_CORR_OFFSET;
	writel(tmp, priv->hwp + FEC_ATIME_INC);
	writel(tc.corr_period, priv->hwp + FEC_ATIME_CORR);
}

/**
 * fec_ptp_read - read raw cycle counter (to be used by time counter)
 * @cc: the cyclecounter structure
 *
 * this function reads the cyclecounter registers and is called by the
 * cyclecounter structure used to construct a ns counter from the
 * arbitrary fixed point registers
 */
static cycle_t fec_ptp_read(const struct cyclecounter *cc)
{
	struct fec_enet_private *fep =
		container_of(cc, struct fec_enet_private, cc);
	u32 tempval;

	tempval = readl(fep->hwp + FEC_ATIME_CTRL);
	tempval |= FEC_T_CTRL_CAPTURE;
	writel(tempval, fep->hwp + FEC_ATIME_CTRL);

	return readl(fep->hwp + FEC_ATIME);
}

/**
 * fec_ptp_start_cyclecounter - create the cycle counter from hw
 * @ndev: network device
 *
 * this function initializes the timecounter and cyclecounter
 * structures for use in generated a ns counter from the arbitrary
 * fixed point cycles registers in the hardware.
 */
void fec_ptp_start_cyclecounter(struct net_device *ndev)
{
	struct fec_enet_private *fep = netdev_priv(ndev);
	unsigned long flags;
	int inc;

	inc = FEC_T_PERIOD_ONE_SEC / fep->cycle_speed;

	/* grab the ptp lock */
	spin_lock_irqsave(&fep->tmreg_lock, flags);
	writel(FEC_T_CTRL_RESTART, fep->hwp + FEC_ATIME_CTRL);

	/* 1ns counter */
	writel(inc << FEC_T_INC_OFFSET, fep->hwp + FEC_ATIME_INC);

	if (fep->hwts_rx_en_ioctl || fep->hwts_tx_en_ioctl) {
		writel(FEC_T_PERIOD_ONE_SEC, fep->hwp + FEC_ATIME_EVT_PERIOD);
		/* start counter */
		writel(FEC_T_CTRL_PERIOD_RST | FEC_T_CTRL_ENABLE,
				fep->hwp + FEC_ATIME_CTRL);
	} else if (fep->hwts_tx_en || fep->hwts_tx_en) {
		/* use free running count */
		writel(0, fep->hwp + FEC_ATIME_EVT_PERIOD);
		writel(FEC_T_CTRL_ENABLE, fep->hwp + FEC_ATIME_CTRL);
	}

	memset(&fep->cc, 0, sizeof(fep->cc));
	fep->cc.read = fec_ptp_read;
	fep->cc.mask = CLOCKSOURCE_MASK(32);
	fep->cc.shift = 31;
	fep->cc.mult = FEC_CC_MULT;

	/* reset the ns time counter */
	timecounter_init(&fep->tc, &fep->cc, ktime_to_ns(ktime_get_real()));

	spin_unlock_irqrestore(&fep->tmreg_lock, flags);
}

/**
 * fec_ptp_adjfreq - adjust ptp cycle frequency
 * @ptp: the ptp clock structure
 * @ppb: parts per billion adjustment from base
 *
 * Adjust the frequency of the ptp cycle counter by the
 * indicated ppb from the base frequency.
 *
 * Because ENET hardware frequency adjust is complex,
 * using software method to do that.
 */
static int fec_ptp_adjfreq(struct ptp_clock_info *ptp, s32 ppb)
{
	u64 diff;
	unsigned long flags;
	int neg_adj = 0;
	u32 mult = FEC_CC_MULT;

	struct fec_enet_private *fep =
	    container_of(ptp, struct fec_enet_private, ptp_caps);

	if (ppb < 0) {
		ppb = -ppb;
		neg_adj = 1;
	}

	diff = mult;
	diff *= ppb;
	diff = div_u64(diff, 1000000000ULL);

	spin_lock_irqsave(&fep->tmreg_lock, flags);
	/*
	 * dummy read to set cycle_last in tc to now.
	 * So use adjusted mult to calculate when next call
	 * timercounter_read.
	 */
	timecounter_read(&fep->tc);

	fep->cc.mult = neg_adj ? mult - diff : mult + diff;

	spin_unlock_irqrestore(&fep->tmreg_lock, flags);

	return 0;
}

/**
 * fec_ptp_adjtime
 * @ptp: the ptp clock structure
 * @delta: offset to adjust the cycle counter by
 *
 * adjust the timer by resetting the timecounter structure.
 */
static int fec_ptp_adjtime(struct ptp_clock_info *ptp, s64 delta)
{
	struct fec_enet_private *fep =
	    container_of(ptp, struct fec_enet_private, ptp_caps);
	unsigned long flags;
	u64 now;

	spin_lock_irqsave(&fep->tmreg_lock, flags);

	now = timecounter_read(&fep->tc);
	now += delta;

	/* reset the timecounter */
	timecounter_init(&fep->tc, &fep->cc, now);

	spin_unlock_irqrestore(&fep->tmreg_lock, flags);

	return 0;
}

/**
 * fec_ptp_gettime
 * @ptp: the ptp clock structure
 * @ts: timespec structure to hold the current time value
 *
 * read the timecounter and return the correct value on ns,
 * after converting it into a struct timespec.
 */
static int fec_ptp_gettime(struct ptp_clock_info *ptp, struct timespec *ts)
{
	struct fec_enet_private *adapter =
	    container_of(ptp, struct fec_enet_private, ptp_caps);
	u64 ns;
	u32 remainder;
	unsigned long flags;

	spin_lock_irqsave(&adapter->tmreg_lock, flags);
	ns = timecounter_read(&adapter->tc);
	spin_unlock_irqrestore(&adapter->tmreg_lock, flags);

	ts->tv_sec = div_u64_rem(ns, 1000000000ULL, &remainder);
	ts->tv_nsec = remainder;

	return 0;
}

/**
 * fec_ptp_settime
 * @ptp: the ptp clock structure
 * @ts: the timespec containing the new time for the cycle counter
 *
 * reset the timecounter to use a new base value instead of the kernel
 * wall timer value.
 */
static int fec_ptp_settime(struct ptp_clock_info *ptp,
			   const struct timespec *ts)
{
	struct fec_enet_private *fep =
	    container_of(ptp, struct fec_enet_private, ptp_caps);

	u64 ns;
	unsigned long flags;

	ns = ts->tv_sec * 1000000000ULL;
	ns += ts->tv_nsec;

	spin_lock_irqsave(&fep->tmreg_lock, flags);
	timecounter_init(&fep->tc, &fep->cc, ns);
	spin_unlock_irqrestore(&fep->tmreg_lock, flags);
	return 0;
}

/**
 * fec_ptp_enable
 * @ptp: the ptp clock structure
 * @rq: the requested feature to change
 * @on: whether to enable or disable the feature
 *
 */
static int fec_ptp_enable(struct ptp_clock_info *ptp,
			  struct ptp_clock_request *rq, int on)
{
	return -EOPNOTSUPP;
}

/**
 * fec_ptp_hwtstamp_ioctl - control hardware time stamping
 * @ndev: pointer to net_device
 * @ifreq: ioctl data
 * @cmd: particular ioctl requested
 */
int fec_ptp_ioctl(struct net_device *ndev, struct ifreq *ifr, int cmd)
{
	struct fec_enet_private *fep = netdev_priv(ndev);
	struct hwtstamp_config config;
	struct ptp_rtc_time curr_time;
	struct ptp_time rx_time, tx_time;
	struct fec_ptp_ts_data p_ts;
	struct fec_ptp_ts_data *p_ts_user;
	struct ptp_set_comp p_comp;
	u32 freq_compensation;
	int retval = 0;

	switch (cmd) {
	case SIOCSHWTSTAMP:
		if (copy_from_user(&config, ifr->ifr_data, sizeof(config)))
			return -EFAULT;

		/* reserved for future extensions */
		if (config.flags)
			return -EINVAL;

		switch (config.tx_type) {
		case HWTSTAMP_TX_OFF:
			fep->hwts_tx_en = 0;
			break;
		case HWTSTAMP_TX_ON:
			fep->hwts_tx_en = 1;
			fep->hwts_tx_en_ioctl = 0;
			break;
		default:
			return -ERANGE;
		}

		switch (config.rx_filter) {
		case HWTSTAMP_FILTER_NONE:
			if (fep->hwts_rx_en)
				fep->hwts_rx_en = 0;
			config.rx_filter = HWTSTAMP_FILTER_NONE;
			break;

		default:
			/*
			 * register RXMTRL must be set in order
			 * to do V1 packets, therefore it is not
			 * possible to time stamp both V1 Sync and
			 * Delay_Req messages and hardware does not support
			 * timestamping all packets => return error
			 */
			fep->hwts_rx_en = 1;
			fep->hwts_rx_en_ioctl = 0;
			config.rx_filter = HWTSTAMP_FILTER_ALL;
			break;
		}

		fec_ptp_start_cyclecounter(ndev);
		return copy_to_user(ifr->ifr_data, &config, sizeof(config)) ?
		    -EFAULT : 0;
		break;
	case PTP_ENBL_TXTS_IOCTL:
	case PTP_ENBL_RXTS_IOCTL:
		fep->hwts_rx_en_ioctl = 1;
		fep->hwts_tx_en_ioctl = 1;
		fep->hwts_rx_en = 0;
		fep->hwts_tx_en = 0;
		fec_ptp_start_cyclecounter(ndev);
		break;
	case PTP_DSBL_RXTS_IOCTL:
	case PTP_DSBL_TXTS_IOCTL:
		fep->hwts_rx_en_ioctl = 0;
		fep->hwts_tx_en_ioctl = 0;
		break;
	case PTP_GET_RX_TIMESTAMP:
		p_ts_user = (struct fec_ptp_ts_data *)ifr->ifr_data;
		if (0 != copy_from_user(&p_ts.ident,
			&p_ts_user->ident, sizeof(p_ts.ident)))
			return -EINVAL;
		retval = fec_ptp_find_and_remove(&fep->rx_timestamps,
				&p_ts.ident, &rx_time);
		if (retval == 0 &&
			copy_to_user((void __user *)(&p_ts_user->ts),
				&rx_time, sizeof(rx_time)))
			return -EFAULT;
		break;
	case PTP_GET_TX_TIMESTAMP:
		p_ts_user = (struct fec_ptp_ts_data *)ifr->ifr_data;
		if (0 != copy_from_user(&p_ts.ident,
			&p_ts_user->ident, sizeof(p_ts.ident)))
			return -EINVAL;
		retval = fec_ptp_find_and_remove(&fep->tx_timestamps,
				&p_ts.ident, &tx_time);
		if (retval == 0 &&
			copy_to_user((void __user *)(&p_ts_user->ts),
				&tx_time, sizeof(tx_time)))
			retval = -EFAULT;
		break;
	case PTP_GET_CURRENT_TIME:
		fec_get_curr_cnt(fep, &curr_time);
		if (0 != copy_to_user(ifr->ifr_data,
					&(curr_time.rtc_time),
					sizeof(struct ptp_time)))
			return -EFAULT;
		break;
	case PTP_SET_RTC_TIME:
		if (0 != copy_from_user(&(curr_time.rtc_time),
					ifr->ifr_data,
					sizeof(struct ptp_time)))
			return -EINVAL;
		fec_set_1588cnt(fep, &curr_time);
		break;
	case PTP_FLUSH_TIMESTAMP:
		/* reset tx-timestamping buffer */
		fep->tx_timestamps.front = 0;
		fep->tx_timestamps.end = 0;
		fep->tx_timestamps.size = (DEFAULT_PTP_TX_BUF_SZ + 1);
		/* reset rx-timestamping buffer */
		fep->rx_timestamps.front = 0;
		fep->rx_timestamps.end = 0;
		fep->rx_timestamps.size = (DEFAULT_PTP_RX_BUF_SZ + 1);
		break;
	case PTP_SET_COMPENSATION:
		if (0 != copy_from_user(&p_comp, ifr->ifr_data,
			sizeof(struct ptp_set_comp)))
			return -EINVAL;
		fec_set_drift(fep, &p_comp);
		break;
	case PTP_GET_ORIG_COMP:
		freq_compensation = FEC_PTP_ORIG_COMP;
		if (copy_to_user(ifr->ifr_data, &freq_compensation,
					sizeof(freq_compensation)) > 0)
			return -EFAULT;
		break;
	default:
		return -EINVAL;
	}

	return retval;
}
/**
 * fec_time_keep - call timecounter_read every second to avoid timer overrun
 *                 because ENET just support 32bit counter, will timeout in 4s
 */
static void fec_time_keep(unsigned long _data)
{
	struct fec_enet_private *fep = (struct fec_enet_private *)_data;
	u64 ns;
	unsigned long flags;

	spin_lock_irqsave(&fep->tmreg_lock, flags);
	ns = timecounter_read(&fep->tc);
	spin_unlock_irqrestore(&fep->tmreg_lock, flags);

	mod_timer(&fep->time_keep, jiffies + HZ);
}

/**
 * fec_ptp_init
 * @ndev: The FEC network adapter
 *
 * This function performs the required steps for enabling ptp
 * support. If ptp support has already been loaded it simply calls the
 * cyclecounter init routine and exits.
 */

void fec_ptp_init(struct platform_device *pdev)
{
	struct net_device *ndev = platform_get_drvdata(pdev);
	struct fec_enet_private *fep = netdev_priv(ndev);

	fep->ptp_caps.owner = THIS_MODULE;
	snprintf(fep->ptp_caps.name, 16, "fec ptp");

	fep->ptp_caps.max_adj = 250000000;
	fep->ptp_caps.n_alarm = 0;
	fep->ptp_caps.n_ext_ts = 0;
	fep->ptp_caps.n_per_out = 0;
	fep->ptp_caps.pps = 0;
	fep->ptp_caps.adjfreq = fec_ptp_adjfreq;
	fep->ptp_caps.adjtime = fec_ptp_adjtime;
	fep->ptp_caps.gettime = fec_ptp_gettime;
	fep->ptp_caps.settime = fec_ptp_settime;
	fep->ptp_caps.enable = fec_ptp_enable;

	fep->cycle_speed = clk_get_rate(fep->clk_ptp);

	spin_lock_init(&fep->tmreg_lock);

	fec_ptp_start_cyclecounter(ndev);

	init_timer(&fep->time_keep);
	fep->time_keep.data = (unsigned long)fep;
	fep->time_keep.function = fec_time_keep;
	fep->time_keep.expires = jiffies + HZ;
	add_timer(&fep->time_keep);

	fep->ptp_clock = ptp_clock_register(&fep->ptp_caps, &pdev->dev);
	if (IS_ERR(fep->ptp_clock)) {
		fep->ptp_clock = NULL;
		pr_err("ptp_clock_register failed\n");
	}

	/* initialize circular buffer for tx timestamps */
	if (fec_ptp_init_circ(&(fep->tx_timestamps),
		(DEFAULT_PTP_TX_BUF_SZ+1)))
		pr_err("init tx circular buffer failed\n");
	/* initialize circular buffer for rx timestamps */
	if (fec_ptp_init_circ(&(fep->rx_timestamps),
			(DEFAULT_PTP_RX_BUF_SZ+1)))
		pr_err("init rx curcular buffer failed\n");
}

void fec_ptp_cleanup(struct fec_enet_private *priv)
{
	if (priv->tx_timestamps.data_buf)
		vfree(priv->tx_timestamps.data_buf);
	if (priv->rx_timestamps.data_buf)
		vfree(priv->rx_timestamps.data_buf);
}

