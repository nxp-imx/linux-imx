/*
 * Copyright (c) 2009-2010 Atheros Communications Inc.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#include <linux/module.h>
#include <linux/kernel.h>

#include <linux/init.h>
#include <linux/slab.h>
#include <linux/tty.h>
#include <linux/errno.h>
#include <linux/ioctl.h>
#include <linux/skbuff.h>

#include <net/bluetooth/bluetooth.h>
#include <net/bluetooth/hci_core.h>

#include "hci_uart.h"

#ifdef DEBUG
#define ATH_DBG(fmt, arg...)   printk(KERN_ERR "[ATH_DBG] (%s) <%s>: " fmt "\n" , __FILE__ , __func__ , ## arg)
#define ATH_INFO(fmt, arg...)  printk(KERN_INFO "[ATH_DBG] (%s) <%s>: " fmt "\n" , __FILE__ , __func__ , ## arg)
#else
#define ATH_DBG(fmt, arg...)
#define ATH_INFO(fmt, arg...)
#endif


/* HCIATH receiver States */
#define HCIATH_W4_PACKET_TYPE			0
#define HCIATH_W4_EVENT_HDR			1
#define HCIATH_W4_ACL_HDR			2
#define HCIATH_W4_SCO_HDR			3
#define HCIATH_W4_DATA				4

struct ath_struct {
	struct hci_uart *hu;
	unsigned int rx_state;
	unsigned int rx_count;
	unsigned int cur_sleep;

	spinlock_t hciath_lock;
	struct sk_buff *rx_skb;
	struct sk_buff_head txq;
	wait_queue_head_t wqevt;
	struct work_struct ctxtsw;
};

int ath_wakeup_ar3001(struct tty_struct *tty)
{
	struct termios settings;
	int status = 0x00;
	mm_segment_t oldfs;
	status = tty->driver->ops->tiocmget(tty, NULL);

	ATH_DBG("");

	if ((status & TIOCM_CTS))
		return status;

	oldfs = get_fs();
	set_fs(KERNEL_DS);
	n_tty_ioctl_helper(tty, NULL, TCGETS, (unsigned long)&settings);

	settings.c_cflag &= ~CRTSCTS;
	n_tty_ioctl_helper(tty, NULL, TCSETS, (unsigned long)&settings);
	set_fs(oldfs);
	status = tty->driver->ops->tiocmget(tty, NULL);

	/* Wake up board */
	tty->driver->ops->tiocmset(tty, NULL, 0x00, TIOCM_RTS);
	mdelay(20);

	status = tty->driver->ops->tiocmget(tty, NULL);

	tty->driver->ops->tiocmset(tty, NULL, TIOCM_RTS, 0x00);
	mdelay(20);

	status = tty->driver->ops->tiocmget(tty, NULL);
	oldfs = get_fs();
	set_fs(KERNEL_DS);
	n_tty_ioctl_helper(tty, NULL, TCGETS, (unsigned long)&settings);

	settings.c_cflag |= CRTSCTS;
	n_tty_ioctl_helper(tty, NULL, TCSETS, (unsigned long)&settings);
	set_fs(oldfs);
	return status;
}

static void ath_context_switch(struct work_struct *work)
{
	int status;
	struct ath_struct *ath;
	struct hci_uart *hu;
	struct tty_struct *tty;

	ATH_DBG("");

	ath = container_of(work, struct ath_struct, ctxtsw);

	hu = ath->hu;
	tty = hu->tty;

	/* verify and wake up controller */
	if (ath->cur_sleep) {

		status = ath_wakeup_ar3001(tty);
		if (!(status & TIOCM_CTS))
			return;
	}

	/* Ready to send Data */
	clear_bit(HCI_UART_SENDING, &hu->tx_state);
	hci_uart_tx_wakeup(hu);
}

int ath_check_sleep_cmd(struct ath_struct *ath, unsigned char *packet)
{
	ATH_DBG("");

	if (packet[0] == 0x04 && packet[1] == 0xFC)
		ath->cur_sleep = packet[3];

	ATH_DBG("ath->cur_sleep:%d\n", ath->cur_sleep);

	return 0;
}


/* Initialize protocol */
static int ath_open(struct hci_uart *hu)
{
	struct ath_struct *ath;
	BT_DBG("hu %p", hu);
	ATH_INFO("hu %p", hu);

	ath = kzalloc(sizeof(*ath), GFP_ATOMIC);
	if (!ath)
		return -ENOMEM;

	skb_queue_head_init(&ath->txq);
	spin_lock_init(&ath->hciath_lock);

	ath->cur_sleep = 0;
	hu->priv = ath;
	ath->hu = hu;

	init_waitqueue_head(&ath->wqevt);
	INIT_WORK(&ath->ctxtsw, ath_context_switch);
	return 0;
}

/* Flush protocol data */
static int ath_flush(struct hci_uart *hu)
{
	struct ath_struct *ath = hu->priv;
	BT_DBG("hu %p", hu);
	ATH_INFO("hu %p", hu);
	skb_queue_purge(&ath->txq);

	return 0;
}

/* Close protocol */
static int ath_close(struct hci_uart *hu)
{
	struct ath_struct *ath = hu->priv;
	BT_DBG("hu %p", hu);
	ATH_INFO("hu %p", hu);

	skb_queue_purge(&ath->txq);

	if (ath->rx_skb)
		kfree_skb(ath->rx_skb);

	wake_up_interruptible(&ath->wqevt);
	hu->priv = NULL;
	kfree(ath);
	return 0;
}

/* Enqueue frame for transmittion */
static int ath_enqueue(struct hci_uart *hu, struct sk_buff *skb)
{
	struct ath_struct *ath = hu->priv;
	if (bt_cb(skb)->pkt_type == HCI_SCODATA_PKT) {

		/* Discard SCO packet.AR3001 does not support SCO over HCI */
		BT_DBG("SCO Packet over HCI received Dropping\n");
		kfree(skb);
		return 0;
	}
	BT_DBG("hu %p skb %p", hu, skb);
	ATH_DBG("hu %p skb %p", hu, skb);

	/* Prepend skb with frame type */
	memcpy(skb_push(skb, 1), &bt_cb(skb)->pkt_type, 1);

	skb_queue_tail(&ath->txq, skb);
	set_bit(HCI_UART_SENDING, &hu->tx_state);

	schedule_work(&ath->ctxtsw);
	return 0;
}

static struct sk_buff *ath_dequeue(struct hci_uart *hu)
{
	struct ath_struct *ath = hu->priv;
	struct sk_buff *skbuf;

	ATH_DBG("");

	skbuf = skb_dequeue(&ath->txq);
	if (skbuf != NULL)
		ath_check_sleep_cmd(ath, &skbuf->data[1]);

	return skbuf;
}

static inline int ath_check_data_len(struct ath_struct *ath, int len)
{
	register int room = skb_tailroom(ath->rx_skb);
	BT_DBG("len %d room %d", len, room);
	ATH_DBG("len %d room %d", len, room);

	if (len > room) {
		BT_ERR("Data length is too large");
		kfree_skb(ath->rx_skb);
		ath->rx_state = HCIATH_W4_PACKET_TYPE;
		ath->rx_skb = NULL;
		ath->rx_count = 0;
	} else {
		ath->rx_state = HCIATH_W4_DATA;
		ath->rx_count = len;
		return len;
	}

	return 0;
}

/* Recv data */
static int ath_recv(struct hci_uart *hu, void *data, int count)
{
	struct ath_struct *ath = hu->priv;
	register char *ptr;
	struct hci_event_hdr *eh;
	struct hci_acl_hdr *ah;
	struct hci_sco_hdr *sh;
	struct sk_buff *skbuf;
	register int len, type, dlen;

	skbuf = NULL;
	BT_DBG("hu %p count %d rx_state %d rx_count %d", hu, count,
	       ath->rx_state, ath->rx_count);
	ATH_DBG("hu %p count %d rx_state %d rx_count %d", hu, count,
	       ath->rx_state, ath->rx_count);
	ptr = data;
	while (count) {
		if (ath->rx_count) {

			len = min_t(unsigned int, ath->rx_count, count);
			memcpy(skb_put(ath->rx_skb, len), ptr, len);
			ath->rx_count -= len;
			count -= len;
			ptr += len;

			if (ath->rx_count)
				continue;
			switch (ath->rx_state) {
			case HCIATH_W4_DATA:
				hci_recv_frame(ath->rx_skb);
				ath->rx_state = HCIATH_W4_PACKET_TYPE;
				ath->rx_skb = NULL;
				ath->rx_count = 0;
				continue;
			case HCIATH_W4_EVENT_HDR:
				eh = (struct hci_event_hdr *)ath->rx_skb->data;
				BT_DBG("Event header: evt 0x%2.2x plen %d",
				       eh->evt, eh->plen);
				ATH_DBG("Event header: evt 0x%2.2x plen %d",
				       eh->evt, eh->plen);
				ath_check_data_len(ath, eh->plen);
				continue;
			case HCIATH_W4_ACL_HDR:
				ah = (struct hci_acl_hdr *)ath->rx_skb->data;
				dlen = __le16_to_cpu(ah->dlen);
				BT_DBG("ACL header: dlen %d", dlen);
				ATH_DBG("ACL header: dlen %d", dlen);
				ath_check_data_len(ath, dlen);
				continue;
			case HCIATH_W4_SCO_HDR:
				sh = (struct hci_sco_hdr *)ath->rx_skb->data;
				BT_DBG("SCO header: dlen %d", sh->dlen);
				ATH_DBG("SCO header: dlen %d", sh->dlen);
				ath_check_data_len(ath, sh->dlen);
				continue;
			}
		}

		/* HCIATH_W4_PACKET_TYPE */
		switch (*ptr) {
		case HCI_EVENT_PKT:
			BT_DBG("Event packet");
			ATH_DBG("Event packet");
			ath->rx_state = HCIATH_W4_EVENT_HDR;
			ath->rx_count = HCI_EVENT_HDR_SIZE;
			type = HCI_EVENT_PKT;
			break;
		case HCI_ACLDATA_PKT:
			BT_DBG("ACL packet");
			ATH_DBG("ACL packet");
			ath->rx_state = HCIATH_W4_ACL_HDR;
			ath->rx_count = HCI_ACL_HDR_SIZE;
			type = HCI_ACLDATA_PKT;
			break;
		case HCI_SCODATA_PKT:
			BT_DBG("SCO packet");
			ATH_DBG("SCO packet");
			ath->rx_state = HCIATH_W4_SCO_HDR;
			ath->rx_count = HCI_SCO_HDR_SIZE;
			type = HCI_SCODATA_PKT;
			break;
		default:
			BT_ERR("Unknown HCI packet type %2.2x", (__u8) *ptr);
			hu->hdev->stat.err_rx++;
			ptr++;
			count--;
			continue;
		};
		ptr++;
		count--;

		/* Allocate packet */
		ath->rx_skb = bt_skb_alloc(HCI_MAX_FRAME_SIZE, GFP_ATOMIC);
		if (!ath->rx_skb) {
			BT_ERR("Can't allocate mem for new packet");
			ath->rx_state = HCIATH_W4_PACKET_TYPE;
			ath->rx_count = 0;
			return -ENOMEM;
		}
		ath->rx_skb->dev = (void *)hu->hdev;
		bt_cb(ath->rx_skb)->pkt_type = type;
	} return count;
}

static struct hci_uart_proto athp = {
	.id = HCI_UART_ATH,
	.open = ath_open,
	.close = ath_close,
	.recv = ath_recv,
	.enqueue = ath_enqueue,
	.dequeue = ath_dequeue,
	.flush = ath_flush,
};

int ath_init(void)
{
	int err = hci_uart_register_proto(&athp);
	ATH_INFO("");
	if (!err)
		BT_INFO("HCIATH protocol initialized");
	else
		BT_ERR("HCIATH protocol registration failed with err %d", err);
	return err;
}

int ath_deinit(void)
{
	ATH_INFO("");
	return hci_uart_unregister_proto(&athp);
}
