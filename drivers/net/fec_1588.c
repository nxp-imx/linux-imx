/*
 * drivers/net/fec_1588.c
 *
 * Copyright (C) 2010 Freescale Semiconductor, Inc.
 * Copyright (C) 2009 IXXAT Automation, GmbH
 *
 * FEC Ethernet Driver -- IEEE 1588 interface functionality
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

#include <linux/io.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/vmalloc.h>
#include <linux/spinlock.h>
#include <linux/ip.h>
#include <linux/udp.h>
#include "fec.h"
#include "fec_1588.h"

static DECLARE_WAIT_QUEUE_HEAD(ptp_rx_ts_wait);
static DECLARE_WAIT_QUEUE_HEAD(ptp_tx_ts_wait);
#define PTP_GET_RX_TIMEOUT      (HZ/10)
#define PTP_GET_TX_TIMEOUT      (HZ/100)

static struct fec_ptp_private *ptp_private[2];

/* Alloc the ring resource */
static int fec_ptp_init_circ(struct circ_buf *ptp_buf, int size)
{
	ptp_buf->buf = vmalloc(size * sizeof(struct fec_ptp_data_t));

	if (!ptp_buf->buf)
		return 1;
	ptp_buf->head = 0;
	ptp_buf->tail = 0;

	return 0;
}

static inline int fec_ptp_calc_index(int size, int curr_index, int offset)
{
	return (curr_index + offset) % size;
}

static int fec_ptp_is_empty(struct circ_buf *buf)
{
	return (buf->head == buf->tail);
}

static int fec_ptp_nelems(struct circ_buf *buf, int size)
{
	const int front = buf->head;
	const int end = buf->tail;
	int n_items;

	if (end > front)
		n_items = end - front;
	else if (end < front)
		n_items = size - (front - end);
	else
		n_items = 0;

	return n_items;
}

static int fec_ptp_is_full(struct circ_buf *buf, int size)
{
	if (fec_ptp_nelems(buf, size) ==
				(size - 1))
		return 1;
	else
		return 0;
}

static int fec_ptp_insert(struct circ_buf *ptp_buf,
			  struct fec_ptp_data_t *data,
			  struct fec_ptp_private *priv,
			  int size)
{
	struct fec_ptp_data_t *tmp;

	if (fec_ptp_is_full(ptp_buf, size))
		return 1;

	spin_lock(&priv->ptp_lock);
	tmp = (struct fec_ptp_data_t *)(ptp_buf->buf) + ptp_buf->tail;

	tmp->key = data->key;
	memcpy(tmp->spid, data->spid, 10);
	tmp->ts_time.sec = data->ts_time.sec;
	tmp->ts_time.nsec = data->ts_time.nsec;

	ptp_buf->tail = fec_ptp_calc_index(size, ptp_buf->tail, 1);
	spin_unlock(&priv->ptp_lock);

	return 0;
}

static int fec_ptp_find_and_remove(struct circ_buf *ptp_buf,
				   struct fec_ptp_data_t *data,
				   struct fec_ptp_private *priv,
				   int size)
{
	int i;
	int end = ptp_buf->tail;
	unsigned long flags;
	struct fec_ptp_data_t *tmp;

	if (fec_ptp_is_empty(ptp_buf))
		return 1;

	i = ptp_buf->head;
	while (i != end) {
		tmp = (struct fec_ptp_data_t *)(ptp_buf->buf) + i;
		if (tmp->key == data->key &&
				!memcmp(tmp->spid, data->spid, 10))
			break;
		i = fec_ptp_calc_index(size, i, 1);
	}

	spin_lock_irqsave(&priv->ptp_lock, flags);
	if (i == end) {
		ptp_buf->head = end;
		spin_unlock_irqrestore(&priv->ptp_lock, flags);
		return 1;
	}

	data->ts_time.sec = tmp->ts_time.sec;
	data->ts_time.nsec = tmp->ts_time.nsec;

	ptp_buf->head = fec_ptp_calc_index(size, i, 1);
	spin_unlock_irqrestore(&priv->ptp_lock, flags);

	return 0;
}

/* 1588 Module intialization */
int fec_ptp_start(struct fec_ptp_private *priv)
{
	struct fec_ptp_private *fpp = priv;

	/* Select 1588 Timer source and enable module for starting Tmr Clock *
	 * When enable both FEC0 and FEC1 1588 Timer in the same time,       *
	 * enable FEC1 timer's slave mode. */
	if ((fpp == ptp_private[0]) || !(ptp_private[0]->ptp_active)) {
		writel(FEC_T_CTRL_RESTART, fpp->hwp + FEC_ATIME_CTRL);
		writel(FEC_T_INC_40MHZ << FEC_T_INC_OFFSET,
				fpp->hwp + FEC_ATIME_INC);
		writel(FEC_T_PERIOD_ONE_SEC, fpp->hwp + FEC_ATIME_EVT_PERIOD);
		/* start counter */
		writel(FEC_T_CTRL_PERIOD_RST | FEC_T_CTRL_ENABLE,
				fpp->hwp + FEC_ATIME_CTRL);
		fpp->ptp_slave = 0;
		fpp->ptp_active = 1;
		/* if the FEC1 timer was enabled, set it to slave mode */
		if ((fpp == ptp_private[0]) && (ptp_private[1]->ptp_active)) {
			writel(0, ptp_private[1]->hwp + FEC_ATIME_CTRL);
			fpp->prtc = ptp_private[1]->prtc;
			writel(FEC_T_CTRL_RESTART,
					ptp_private[1]->hwp + FEC_ATIME_CTRL);
			writel(FEC_T_INC_40MHZ << FEC_T_INC_OFFSET,
					ptp_private[1]->hwp + FEC_ATIME_INC);
			/* Set the timer as slave mode */
			writel(FEC_T_CTRL_SLAVE,
					ptp_private[1]->hwp + FEC_ATIME_CTRL);
			ptp_private[1]->ptp_slave = 1;
			ptp_private[1]->ptp_active = 1;
		}
	} else {
		writel(FEC_T_INC_40MHZ << FEC_T_INC_OFFSET,
				fpp->hwp + FEC_ATIME_INC);
		/* Set the timer as slave mode */
		writel(FEC_T_CTRL_SLAVE, fpp->hwp + FEC_ATIME_CTRL);
		fpp->ptp_slave = 1;
		fpp->ptp_active = 1;
	}

	return 0;
}

/* Cleanup routine for 1588 module.
 * When PTP is disabled this routing is called */
void fec_ptp_stop(struct fec_ptp_private *priv)
{
	struct fec_ptp_private *fpp = priv;

	writel(0, fpp->hwp + FEC_ATIME_CTRL);
	writel(FEC_T_CTRL_RESTART, fpp->hwp + FEC_ATIME_CTRL);
	priv->ptp_active = 0;
	priv->ptp_slave = 0;

}

static void fec_get_curr_cnt(struct fec_ptp_private *priv,
			struct ptp_rtc_time *curr_time)
{
	u32 tempval;
	struct fec_ptp_private *tmp_priv;

	if (!priv->ptp_slave)
		tmp_priv = priv;
	else
		tmp_priv = ptp_private[0];

	writel(FEC_T_CTRL_CAPTURE, priv->hwp + FEC_ATIME_CTRL);
	writel(FEC_T_CTRL_CAPTURE, priv->hwp + FEC_ATIME_CTRL);
	curr_time->rtc_time.nsec = readl(priv->hwp + FEC_ATIME);
	curr_time->rtc_time.sec = tmp_priv->prtc;

	writel(FEC_T_CTRL_CAPTURE, priv->hwp + FEC_ATIME_CTRL);
	tempval = readl(priv->hwp + FEC_ATIME);
	if (tempval < curr_time->rtc_time.nsec) {
		curr_time->rtc_time.nsec = tempval;
		curr_time->rtc_time.sec = tmp_priv->prtc;
	}
}

/* Set the 1588 timer counter registers */
static void fec_set_1588cnt(struct fec_ptp_private *priv,
			struct ptp_rtc_time *fec_time)
{
	u32 tempval;
	unsigned long flags;
	struct fec_ptp_private *tmp_priv;

	if (!priv->ptp_slave)
		tmp_priv = priv;
	else
		tmp_priv = ptp_private[0];

	spin_lock_irqsave(&priv->cnt_lock, flags);
	tmp_priv->prtc = fec_time->rtc_time.sec;

	tempval = fec_time->rtc_time.nsec;
	writel(tempval, tmp_priv->hwp + FEC_ATIME);
	spin_unlock_irqrestore(&priv->cnt_lock, flags);
}

/* Set the BD to ptp */
int fec_ptp_do_txstamp(struct sk_buff *skb)
{
	struct iphdr *iph;
	struct udphdr *udph;

	if (skb->len > 44) {
		/* Check if port is 319 for PTP Event, and check for UDP */
		iph = ip_hdr(skb);
		if (iph == NULL || iph->protocol != FEC_PACKET_TYPE_UDP)
			return 0;

		udph = udp_hdr(skb);
		if (udph != NULL && ntohs(udph->dest) == 319)
			return 1;
	}

	return 0;
}

void fec_ptp_store_txstamp(struct fec_ptp_private *priv,
			   struct sk_buff *skb,
			   struct bufdesc *bdp)
{
	int msg_type, seq_id, control;
	struct fec_ptp_data_t tmp_tx_time;
	struct fec_ptp_private *fpp;
	unsigned char *sp_id;
	unsigned short portnum;

	if (!priv->ptp_slave)
		fpp = priv;
	else
		fpp = ptp_private[0];

	seq_id = *((u16 *)(skb->data + FEC_PTP_SEQ_ID_OFFS));
	control = *((u8 *)(skb->data + FEC_PTP_CTRL_OFFS));
	sp_id = skb->data + FEC_PTP_SPORT_ID_OFFS;
	portnum = ntohs(*((unsigned short *)(sp_id + 8)));

	tmp_tx_time.key = ntohs(seq_id);
	memcpy(tmp_tx_time.spid, sp_id, 8);
	memcpy(tmp_tx_time.spid + 8, (unsigned char *)&portnum, 2);
	tmp_tx_time.ts_time.sec = fpp->prtc;
	tmp_tx_time.ts_time.nsec = bdp->ts;

	switch (control) {

	case PTP_MSG_SYNC:
		fec_ptp_insert(&(priv->tx_time_sync), &tmp_tx_time, priv,
				DEFAULT_PTP_TX_BUF_SZ);
		break;

	case PTP_MSG_DEL_REQ:
		fec_ptp_insert(&(priv->tx_time_del_req), &tmp_tx_time, priv,
				DEFAULT_PTP_TX_BUF_SZ);
		break;

	/* clear transportSpecific field*/
	case PTP_MSG_ALL_OTHER:
		msg_type = (*((u8 *)(skb->data +
				FEC_PTP_MSG_TYPE_OFFS))) & 0x0F;
		switch (msg_type) {
		case PTP_MSG_P_DEL_REQ:
			fec_ptp_insert(&(priv->tx_time_pdel_req), &tmp_tx_time,
					priv, DEFAULT_PTP_TX_BUF_SZ);
			break;
		case PTP_MSG_P_DEL_RESP:
			fec_ptp_insert(&(priv->tx_time_pdel_resp), &tmp_tx_time,
					priv, DEFAULT_PTP_TX_BUF_SZ);
			break;
		default:
			break;
		}
		break;
	default:
		break;
	}

	wake_up_interruptible(&ptp_tx_ts_wait);
}

void fec_ptp_store_rxstamp(struct fec_ptp_private *priv,
			   struct sk_buff *skb,
			   struct bufdesc *bdp)
{
	int msg_type, seq_id, control;
	struct fec_ptp_data_t tmp_rx_time;
	struct fec_ptp_private *fpp;
	struct iphdr *iph;
	struct udphdr *udph;
	unsigned char *sp_id;
	unsigned short portnum;

	if (!priv->ptp_slave)
		fpp = priv;
	else
		fpp = ptp_private[0];

	/* Check for UDP, and Check if port is 319 for PTP Event */
	iph = (struct iphdr *)(skb->data + FEC_PTP_IP_OFFS);
	if (iph->protocol != FEC_PACKET_TYPE_UDP)
		return;

	udph = (struct udphdr *)(skb->data + FEC_PTP_UDP_OFFS);
	if (ntohs(udph->dest) != 319)
		return;

	seq_id = *((u16 *)(skb->data + FEC_PTP_SEQ_ID_OFFS));
	control = *((u8 *)(skb->data + FEC_PTP_CTRL_OFFS));
	sp_id = skb->data + FEC_PTP_SPORT_ID_OFFS;
	portnum = ntohs(*((unsigned short *)(sp_id + 8)));

	tmp_rx_time.key = ntohs(seq_id);
	memcpy(tmp_rx_time.spid, sp_id, 8);
	memcpy(tmp_rx_time.spid + 8, (unsigned char *)&portnum, 2);
	tmp_rx_time.ts_time.sec = fpp->prtc;
	tmp_rx_time.ts_time.nsec = bdp->ts;

	switch (control) {

	case PTP_MSG_SYNC:
		fec_ptp_insert(&(priv->rx_time_sync), &tmp_rx_time, priv,
				DEFAULT_PTP_RX_BUF_SZ);
		break;

	case PTP_MSG_DEL_REQ:
		fec_ptp_insert(&(priv->rx_time_del_req), &tmp_rx_time, priv,
				DEFAULT_PTP_RX_BUF_SZ);
		break;

	/* clear transportSpecific field*/
	case PTP_MSG_ALL_OTHER:
		msg_type = (*((u8 *)(skb->data +
				FEC_PTP_MSG_TYPE_OFFS))) & 0x0F;
		switch (msg_type) {
		case PTP_MSG_P_DEL_REQ:
			fec_ptp_insert(&(priv->rx_time_pdel_req), &tmp_rx_time,
					priv, DEFAULT_PTP_RX_BUF_SZ);
			break;
		case PTP_MSG_P_DEL_RESP:
			fec_ptp_insert(&(priv->rx_time_pdel_resp), &tmp_rx_time,
					priv, DEFAULT_PTP_RX_BUF_SZ);
			break;
		default:
			break;
		}
		break;
	default:
		break;
	}

	wake_up_interruptible(&ptp_rx_ts_wait);
}

static uint8_t fec_get_tx_timestamp(struct fec_ptp_private *priv,
				    struct ptp_ts_data *pts,
				    struct ptp_time *tx_time)
{
	struct fec_ptp_data_t tmp;
	int flag;
	u8 mode;

	tmp.key = pts->seq_id;
	memcpy(tmp.spid, pts->spid, 10);
	mode = pts->message_type;
	switch (mode) {
	case PTP_MSG_SYNC:
		flag = fec_ptp_find_and_remove(&(priv->tx_time_sync), &tmp,
						priv, DEFAULT_PTP_TX_BUF_SZ);
		break;
	case PTP_MSG_DEL_REQ:
		flag = fec_ptp_find_and_remove(&(priv->tx_time_del_req), &tmp,
						priv, DEFAULT_PTP_TX_BUF_SZ);
		break;

	case PTP_MSG_P_DEL_REQ:
		flag = fec_ptp_find_and_remove(&(priv->tx_time_pdel_req), &tmp,
						priv, DEFAULT_PTP_TX_BUF_SZ);
		break;
	case PTP_MSG_P_DEL_RESP:
		flag = fec_ptp_find_and_remove(&(priv->tx_time_pdel_resp), &tmp,
						priv, DEFAULT_PTP_TX_BUF_SZ);
		break;

	default:
		flag = 1;
		printk(KERN_ERR "ERROR\n");
		break;
	}

	if (!flag) {
		tx_time->sec = tmp.ts_time.sec;
		tx_time->nsec = tmp.ts_time.nsec;
		return 0;
	} else {
		wait_event_interruptible_timeout(ptp_tx_ts_wait, 0,
					PTP_GET_TX_TIMEOUT);

		switch (mode) {
		case PTP_MSG_SYNC:
			flag = fec_ptp_find_and_remove(&(priv->tx_time_sync),
					&tmp, priv, DEFAULT_PTP_TX_BUF_SZ);
			break;
		case PTP_MSG_DEL_REQ:
			flag = fec_ptp_find_and_remove(&(priv->tx_time_del_req),
					&tmp, priv, DEFAULT_PTP_TX_BUF_SZ);
			break;
		case PTP_MSG_P_DEL_REQ:
			flag = fec_ptp_find_and_remove(
				&(priv->tx_time_pdel_req), &tmp, priv,
				DEFAULT_PTP_TX_BUF_SZ);
			break;
		case PTP_MSG_P_DEL_RESP:
			flag = fec_ptp_find_and_remove(
				&(priv->tx_time_pdel_resp), &tmp, priv,
				DEFAULT_PTP_TX_BUF_SZ);
			break;
		}

		if (flag == 0) {
			tx_time->sec = tmp.ts_time.sec;
			tx_time->nsec = tmp.ts_time.nsec;
			return 0;
		}

		return -1;
	}
}

static uint8_t fec_get_rx_timestamp(struct fec_ptp_private *priv,
				    struct ptp_ts_data *pts,
				    struct ptp_time *rx_time)
{
	struct fec_ptp_data_t tmp;
	int flag;
	u8 mode;

	tmp.key = pts->seq_id;
	memcpy(tmp.spid, pts->spid, 10);
	mode = pts->message_type;
	switch (mode) {
	case PTP_MSG_SYNC:
		flag = fec_ptp_find_and_remove(&(priv->rx_time_sync), &tmp,
				priv, DEFAULT_PTP_RX_BUF_SZ);
		break;
	case PTP_MSG_DEL_REQ:
		flag = fec_ptp_find_and_remove(&(priv->rx_time_del_req), &tmp,
				priv, DEFAULT_PTP_RX_BUF_SZ);
		break;

	case PTP_MSG_P_DEL_REQ:
		flag = fec_ptp_find_and_remove(&(priv->rx_time_pdel_req), &tmp,
				priv, DEFAULT_PTP_RX_BUF_SZ);
		break;
	case PTP_MSG_P_DEL_RESP:
		flag = fec_ptp_find_and_remove(&(priv->rx_time_pdel_resp), &tmp,
				priv, DEFAULT_PTP_RX_BUF_SZ);
		break;

	default:
		flag = 1;
		printk(KERN_ERR "ERROR\n");
		break;
	}

	if (!flag) {
		rx_time->sec = tmp.ts_time.sec;
		rx_time->nsec = tmp.ts_time.nsec;
		return 0;
	} else {
		wait_event_interruptible_timeout(ptp_rx_ts_wait, 0,
					PTP_GET_RX_TIMEOUT);

		switch (mode) {
		case PTP_MSG_SYNC:
			flag = fec_ptp_find_and_remove(&(priv->rx_time_sync),
				&tmp, priv, DEFAULT_PTP_RX_BUF_SZ);
			break;
		case PTP_MSG_DEL_REQ:
			flag = fec_ptp_find_and_remove(
				&(priv->rx_time_del_req), &tmp, priv,
				DEFAULT_PTP_RX_BUF_SZ);
			break;
		case PTP_MSG_P_DEL_REQ:
			flag = fec_ptp_find_and_remove(
				&(priv->rx_time_pdel_req), &tmp, priv,
				DEFAULT_PTP_RX_BUF_SZ);
			break;
		case PTP_MSG_P_DEL_RESP:
			flag = fec_ptp_find_and_remove(
				&(priv->rx_time_pdel_resp), &tmp, priv,
				DEFAULT_PTP_RX_BUF_SZ);
			break;
		}

		if (flag == 0) {
			rx_time->sec = tmp.ts_time.sec;
			rx_time->nsec = tmp.ts_time.nsec;
			return 0;
		}

		return -1;
	}
}

static void fec_handle_ptpdrift(struct ptp_set_comp *comp,
				struct ptp_time_correct *ptc)
{
	u32 ndrift;
	u32 i;
	u32 tmp_current, tmp_winner;

	ndrift = comp->drift;

	if (ndrift == 0) {
		ptc->corr_inc = 0;
		ptc->corr_period = 0;
		return;
	} else if (ndrift >= FEC_ATIME_40MHZ) {
		ptc->corr_inc = (u32)(ndrift / FEC_ATIME_40MHZ);
		ptc->corr_period = 1;
		return;
	} else if ((ndrift < FEC_ATIME_40MHZ) && (comp->o_ops == 0)) {
		tmp_winner = 0xFFFFFFFF;
		for (i = 1; i < 25; i++) {
			tmp_current = (FEC_ATIME_40MHZ * i) % ndrift;
			if (tmp_current == 0) {
				ptc->corr_inc = i;
				ptc->corr_period = (u32)((FEC_ATIME_40MHZ * i)
								/ ndrift);
				break;
			} else if (tmp_current < tmp_winner) {
				ptc->corr_inc = i;
				ptc->corr_period = (u32)((FEC_ATIME_40MHZ * i)
								/ ndrift);
				tmp_winner = tmp_current;
			}
		}
	} else if ((ndrift < FEC_ATIME_40MHZ) && (comp->o_ops == 1)) {
		tmp_winner = 0xFFFFFFFF;
		for (i = 1; i < 100; i++) {
			tmp_current = (FEC_ATIME_40MHZ * i) % ndrift;
			if (tmp_current == 0) {
				ptc->corr_inc = i;
				ptc->corr_period = (u32)((FEC_ATIME_40MHZ * i)
								/ ndrift);
				break;
			} else if (tmp_current < tmp_winner) {
				ptc->corr_inc = i;
				ptc->corr_period = (u32)((FEC_ATIME_40MHZ * i)
								/ ndrift);
				tmp_winner = tmp_current;
			}
		}
	}

}

static void fec_set_drift(struct fec_ptp_private *priv,
			  struct ptp_set_comp *comp)
{
	struct ptp_time_correct	tc;
	struct fec_ptp_private *fpp;
	u32 tmp, corr_ns;

	memset(&tc, 0, sizeof(struct ptp_time_correct));
	fec_handle_ptpdrift(comp, &tc);
	if (tc.corr_inc == 0)
		return;

	if (comp->o_ops == TRUE)
		corr_ns = FEC_T_INC_40MHZ + tc.corr_inc;
	else
		corr_ns = FEC_T_INC_40MHZ - tc.corr_inc;

	if (!priv->ptp_slave)
		fpp = priv;
	else
		fpp = ptp_private[0];
	tmp = readl(fpp->hwp + FEC_ATIME_INC) & FEC_T_INC_MASK;
	tmp |= corr_ns << FEC_T_INC_CORR_OFFSET;
	writel(tmp, fpp->hwp + FEC_ATIME_INC);

	writel(tc.corr_period, fpp->hwp + FEC_ATIME_CORR);
}

static int ptp_open(struct inode *inode, struct file *file)
{
	return 0;
}

static int ptp_release(struct inode *inode, struct file *file)
{
	return 0;
}

static int ptp_ioctl(
	struct inode *inode,
	struct file *file,
	unsigned int cmd,
	unsigned long arg)
{
	struct ptp_rtc_time *cnt;
	struct ptp_rtc_time curr_time;
	struct ptp_time rx_time, tx_time;
	struct ptp_ts_data *p_ts;
	struct ptp_set_comp *p_comp;
	struct fec_ptp_private *priv;
	unsigned int minor = MINOR(inode->i_rdev);
	int retval = 0;

	priv = (struct fec_ptp_private *) ptp_private[minor];
	switch (cmd) {
	case PTP_GET_RX_TIMESTAMP:
		p_ts = (struct ptp_ts_data *)arg;
		retval = fec_get_rx_timestamp(priv, p_ts, &rx_time);
		if (retval == 0)
			copy_to_user((void __user *)(&(p_ts->ts)), &rx_time,
					sizeof(rx_time));
		break;
	case PTP_GET_TX_TIMESTAMP:
		p_ts = (struct ptp_ts_data *)arg;
		fec_get_tx_timestamp(priv, p_ts, &tx_time);
		copy_to_user((void __user *)(&(p_ts->ts)), &tx_time,
				sizeof(tx_time));
		break;
	case PTP_GET_CURRENT_TIME:
		fec_get_curr_cnt(priv, &curr_time);
		copy_to_user((void __user *)arg, &curr_time, sizeof(curr_time));
		break;
	case PTP_SET_RTC_TIME:
		cnt = (struct ptp_rtc_time *)arg;
		fec_set_1588cnt(priv, cnt);
		break;
	case PTP_FLUSH_TIMESTAMP:
		/* reset sync buffer */
		priv->rx_time_sync.head = 0;
		priv->rx_time_sync.tail = 0;
		priv->tx_time_sync.head = 0;
		priv->tx_time_sync.tail = 0;
		/* reset delay_req buffer */
		priv->rx_time_del_req.head = 0;
		priv->rx_time_del_req.tail = 0;
		priv->tx_time_del_req.head = 0;
		priv->tx_time_del_req.tail = 0;
		/* reset pdelay_req buffer */
		priv->rx_time_pdel_req.head = 0;
		priv->rx_time_pdel_req.tail = 0;
		priv->tx_time_pdel_req.head = 0;
		priv->tx_time_pdel_req.tail = 0;
		/* reset pdelay_resp buffer */
		priv->rx_time_pdel_resp.head = 0;
		priv->rx_time_pdel_resp.tail = 0;
		priv->tx_time_pdel_resp.head = 0;
		priv->tx_time_pdel_resp.tail = 0;
		break;
	case PTP_SET_COMPENSATION:
		p_comp = (struct ptp_set_comp *)arg;
		fec_set_drift(priv, p_comp);
		break;
	case PTP_GET_ORIG_COMP:
		((struct ptp_get_comp *)arg)->dw_origcomp = FEC_PTP_ORIG_COMP;
		break;
	default:
		return -EINVAL;
	}
	return retval;
}

static const struct file_operations ptp_fops = {
	.owner	= THIS_MODULE,
	.llseek	= NULL,
	.read	= NULL,
	.write	= NULL,
	.ioctl	= ptp_ioctl,
	.open	= ptp_open,
	.release = ptp_release,
};

static int init_ptp(void)
{
	if (register_chrdev(PTP_MAJOR, "ptp", &ptp_fops))
		printk(KERN_ERR "Unable to register PTP device as char\n");
	else
		printk(KERN_INFO "Register PTP device as char /dev/ptp\n");

	return 0;
}

static void ptp_free(void)
{
	/*unregister the PTP device*/
	unregister_chrdev(PTP_MAJOR, "ptp");
}

/*
 * Resource required for accessing 1588 Timer Registers.
 */
int fec_ptp_init(struct fec_ptp_private *priv, int id)
{
	fec_ptp_init_circ(&(priv->rx_time_sync), DEFAULT_PTP_RX_BUF_SZ);
	fec_ptp_init_circ(&(priv->rx_time_del_req), DEFAULT_PTP_RX_BUF_SZ);
	fec_ptp_init_circ(&(priv->rx_time_pdel_req), DEFAULT_PTP_RX_BUF_SZ);
	fec_ptp_init_circ(&(priv->rx_time_pdel_resp), DEFAULT_PTP_RX_BUF_SZ);
	fec_ptp_init_circ(&(priv->tx_time_sync), DEFAULT_PTP_TX_BUF_SZ);
	fec_ptp_init_circ(&(priv->tx_time_del_req), DEFAULT_PTP_TX_BUF_SZ);
	fec_ptp_init_circ(&(priv->tx_time_pdel_req), DEFAULT_PTP_TX_BUF_SZ);
	fec_ptp_init_circ(&(priv->tx_time_pdel_resp), DEFAULT_PTP_TX_BUF_SZ);

	spin_lock_init(&priv->ptp_lock);
	spin_lock_init(&priv->cnt_lock);
	ptp_private[id] = priv;
	priv->dev_id = id;
	if (id == 0)
		init_ptp();
	return 0;
}
EXPORT_SYMBOL(fec_ptp_init);

void fec_ptp_cleanup(struct fec_ptp_private *priv)
{

	if (priv->rx_time_sync.buf)
		vfree(priv->rx_time_sync.buf);
	if (priv->rx_time_del_req.buf)
		vfree(priv->rx_time_del_req.buf);
	if (priv->rx_time_pdel_req.buf)
		vfree(priv->rx_time_pdel_req.buf);
	if (priv->rx_time_pdel_resp.buf)
		vfree(priv->rx_time_pdel_resp.buf);
	if (priv->tx_time_sync.buf)
		vfree(priv->tx_time_sync.buf);
	if (priv->tx_time_del_req.buf)
		vfree(priv->tx_time_del_req.buf);
	if (priv->tx_time_pdel_req.buf)
		vfree(priv->tx_time_pdel_req.buf);
	if (priv->tx_time_pdel_resp.buf)
		vfree(priv->tx_time_pdel_resp.buf);

	ptp_free();
}
EXPORT_SYMBOL(fec_ptp_cleanup);
