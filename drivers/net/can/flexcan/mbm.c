/*
 * Copyright 2008 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

/*!
 * @file mbm.c
 *
 * @brief Driver for Freescale CAN Controller FlexCAN.
 *
 * @ingroup can
 */
#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/netdevice.h>
#include <linux/if_arp.h>
#include <linux/if_ether.h>
#include <linux/platform_device.h>

#include <asm/io.h>
#include <asm/irq.h>
#include "flexcan.h"

#define flexcan_swab32(x)	\
	(((x) << 24) | ((x) >> 24) |\
		(((x) & (__u32)0x0000ff00UL) << 8) |\
		(((x) & (__u32)0x00ff0000UL) >> 8))

static inline void flexcan_memcpy(void *dst, void *src, int len)
{
	int i;
	unsigned int *d = (unsigned int *)dst, *s = (unsigned int *)src;
	len = (len + 3) >> 2;
	for (i = 0; i < len; i++, s++, d++)
		*d = flexcan_swab32(*s);
}

static void flexcan_mb_bottom(struct net_device *dev, int index)
{
	struct flexcan_device *flexcan = netdev_priv(dev);
	struct net_device_stats *stats = dev->get_stats(dev);
	struct can_hw_mb *hwmb;
	struct can_frame *frame;
	struct sk_buff *skb;
	unsigned int tmp;

	hwmb = flexcan->hwmb + index;
	if (flexcan->fifo || (index >= (flexcan->maxmb - flexcan->xmit_maxmb))) {
		if (hwmb->mb_cs.cs.code == CAN_MB_TX_ABORT)
			hwmb->mb_cs.cs.code = CAN_MB_TX_INACTIVE;

		if (hwmb->mb_cs.cs.code & CAN_MB_TX_INACTIVE) {
			if (netif_queue_stopped(dev))
				netif_start_queue(dev);
			return;
		}
	}
	skb = dev_alloc_skb(sizeof(struct can_frame));
	if (skb) {
		frame = (struct can_frame *)skb_put(skb, sizeof(*frame));
		memset(frame, 0, sizeof(*frame));
		if (hwmb->mb_cs.cs.ide)
			frame->can_id =
			    (hwmb->mb_id & CAN_EFF_MASK) | CAN_EFF_FLAG;
		else
			frame->can_id = (hwmb->mb_id >> 18) & CAN_SFF_MASK;

		if (hwmb->mb_cs.cs.rtr)
			frame->can_id |= CAN_RTR_FLAG;

		frame->can_dlc = hwmb->mb_cs.cs.length;

		if (frame->can_dlc && frame->can_dlc)
			flexcan_memcpy(frame->data, hwmb->mb_data,
				       frame->can_dlc);

		if (flexcan->fifo
		    || (index >= (flexcan->maxmb - flexcan->xmit_maxmb))) {
			hwmb->mb_cs.cs.code = CAN_MB_TX_INACTIVE;
			if (netif_queue_stopped(dev))
				netif_start_queue(dev);
		}

		tmp = __raw_readl(flexcan->io_base + CAN_HW_REG_TIMER);

		dev->last_rx = jiffies;
		stats->rx_packets++;
		stats->rx_bytes += frame->can_dlc;

		skb->dev = dev;
		skb->protocol = __constant_htons(ETH_P_CAN);
		skb->ip_summed = CHECKSUM_UNNECESSARY;
		netif_receive_skb(skb);
	} else {
		tmp = hwmb->mb_cs.data;
		tmp = hwmb->mb_id;
		tmp = hwmb->mb_data[0];
		if (flexcan->fifo
		    || (index >= (flexcan->maxmb - flexcan->xmit_maxmb))) {

			hwmb->mb_cs.cs.code = CAN_MB_TX_INACTIVE;
			if (netif_queue_stopped(dev))
				netif_start_queue(dev);
		}
		tmp = __raw_readl(flexcan->io_base + CAN_HW_REG_TIMER);
		stats->rx_dropped++;
	}
}

static void flexcan_fifo_isr(struct net_device *dev, unsigned int iflag1)
{
	struct flexcan_device *flexcan = dev ? netdev_priv(dev) : NULL;
	struct net_device_stats *stats = dev->get_stats(dev);
	struct sk_buff *skb;
	struct can_hw_mb *hwmb = flexcan->hwmb;
	struct can_frame *frame;
	unsigned int tmp;

	if (iflag1 & __FIFO_RDY_INT) {
		skb = dev_alloc_skb(sizeof(struct can_frame));
		if (skb) {
			frame =
			    (struct can_frame *)skb_put(skb, sizeof(*frame));
			memset(frame, 0, sizeof(*frame));
			if (hwmb->mb_cs.cs.ide)
				frame->can_id =
				    (hwmb->mb_id & CAN_EFF_MASK) | CAN_EFF_FLAG;
			else
				frame->can_id =
				    (hwmb->mb_id >> 18) & CAN_SFF_MASK;

			if (hwmb->mb_cs.cs.rtr)
				frame->can_id |= CAN_RTR_FLAG;

			frame->can_dlc = hwmb->mb_cs.cs.length;

			if (frame->can_dlc && (frame->can_dlc <= 8))
				flexcan_memcpy(frame->data, hwmb->mb_data,
					       frame->can_dlc);
			tmp = __raw_readl(flexcan->io_base + CAN_HW_REG_TIMER);

			dev->last_rx = jiffies;

			stats->rx_packets++;
			stats->rx_bytes += frame->can_dlc;

			skb->dev = dev;
			skb->protocol = __constant_htons(ETH_P_CAN);
			skb->ip_summed = CHECKSUM_UNNECESSARY;
			netif_receive_skb(skb);
		} else {
			tmp = hwmb->mb_cs.data;
			tmp = hwmb->mb_id;
			tmp = hwmb->mb_data[0];
			tmp = __raw_readl(flexcan->io_base + CAN_HW_REG_TIMER);
		}
	}

	if (iflag1 & (__FIFO_OV_INT | __FIFO_WARN_INT)) {
		skb = dev_alloc_skb(sizeof(struct can_frame));
		if (skb) {
			frame =
			    (struct can_frame *)skb_put(skb, sizeof(*frame));
			memset(frame, 0, sizeof(*frame));
			frame->can_id = CAN_ERR_FLAG | CAN_ERR_CRTL;
			frame->can_dlc = CAN_ERR_DLC;
			if (iflag1 & __FIFO_WARN_INT)
				frame->data[1] |=
				    CAN_ERR_CRTL_TX_WARNING |
				    CAN_ERR_CRTL_RX_WARNING;
			if (iflag1 & __FIFO_OV_INT)
				frame->data[1] |= CAN_ERR_CRTL_RX_OVERFLOW;

			skb->dev = dev;
			skb->protocol = __constant_htons(ETH_P_CAN);
			skb->ip_summed = CHECKSUM_UNNECESSARY;
			netif_receive_skb(skb);
		}
	}
}

/*!
 * @brief The function call by CAN ISR to handle mb events.
 *
 * @param dev		the pointer of network device.
 *
 * @return none
 */
void flexcan_mbm_isr(struct net_device *dev)
{
	int i, iflag1, iflag2, maxmb;
	struct flexcan_device *flexcan = dev ? netdev_priv(dev) : NULL;

	if (flexcan->maxmb > 31) {
		maxmb = flexcan->maxmb + 1 - 32;
		iflag1 = __raw_readl(flexcan->io_base + CAN_HW_REG_IFLAG1) &
		    __raw_readl(flexcan->io_base + CAN_HW_REG_IMASK1);
		iflag2 = __raw_readl(flexcan->io_base + CAN_HW_REG_IFLAG2) &
		    __raw_readl(flexcan->io_base + CAN_HW_REG_IMASK2);
		iflag2 &= (1 << maxmb) - 1;
		maxmb = 32;
	} else {
		maxmb = flexcan->maxmb + 1;
		iflag1 = __raw_readl(flexcan->io_base + CAN_HW_REG_IFLAG1) &
		    __raw_readl(flexcan->io_base + CAN_HW_REG_IMASK1);
		iflag1 &= (1 << maxmb) - 1;
		iflag2 = 0;
	}

	__raw_writel(iflag1, flexcan->io_base + CAN_HW_REG_IFLAG1);
	__raw_writel(iflag2, flexcan->io_base + CAN_HW_REG_IFLAG2);

	if (flexcan->fifo) {
		flexcan_fifo_isr(dev, iflag1);
		iflag1 &= 0xFFFFFF00;
	}
	for (i = 0; iflag1 && (i < maxmb); i++) {
		if (iflag1 & (1 << i)) {
			iflag1 &= ~(1 << i);
			flexcan_mb_bottom(dev, i);
		}
	}

	for (i = maxmb; iflag2 && (i <= flexcan->maxmb); i++) {
		if (iflag2 & (1 << (i - 32))) {
			iflag2 &= ~(1 << (i - 32));
			flexcan_mb_bottom(dev, i);
		}
	}
}

/*!
 * @brief function to xmit message buffer
 *
 * @param flexcan	the pointer of can hardware device.
 * @param frame		the pointer of can message frame.
 *
 * @return	Returns 0 if xmit is success. otherwise returns non-zero.
 */
int flexcan_mbm_xmit(struct flexcan_device *flexcan, struct can_frame *frame)
{
	int i = flexcan->xmit_mb;
	struct can_hw_mb *hwmb = flexcan->hwmb;

	do {
		if (hwmb[i].mb_cs.cs.code == CAN_MB_TX_INACTIVE)
			break;
		if ((++i) > flexcan->maxmb) {
			if (flexcan->fifo)
				i = FLEXCAN_MAX_FIFO_MB;
			else
				i = flexcan->xmit_maxmb + 1;
		}
		if (i == flexcan->xmit_mb)
			return -1;
	} while (1);

	flexcan->xmit_mb = i + 1;
	if (flexcan->xmit_mb > flexcan->maxmb) {
		if (flexcan->fifo)
			flexcan->xmit_mb = FLEXCAN_MAX_FIFO_MB;
		else
			flexcan->xmit_mb = flexcan->xmit_maxmb + 1;
	}

	if (frame->can_id & CAN_RTR_FLAG)
		hwmb[i].mb_cs.cs.rtr = 1;
	else
		hwmb[i].mb_cs.cs.rtr = 0;

	if (frame->can_id & CAN_EFF_FLAG) {
		hwmb[i].mb_cs.cs.ide = 1;
		hwmb[i].mb_cs.cs.srr = 1;
		hwmb[i].mb_id = frame->can_id & CAN_EFF_MASK;
	} else {
		hwmb[i].mb_cs.cs.ide = 0;
		hwmb[i].mb_id = (frame->can_id & CAN_SFF_MASK) << 18;
	}

	hwmb[i].mb_cs.cs.length = frame->can_dlc;
	flexcan_memcpy(hwmb[i].mb_data, frame->data, frame->can_dlc);
	hwmb[i].mb_cs.cs.code = CAN_MB_TX_ONCE;
	return 0;
}

/*!
 * @brief function to initial message buffer
 *
 * @param flexcan	the pointer of can hardware device.
 *
 * @return	none
 */
void flexcan_mbm_init(struct flexcan_device *flexcan)
{
	struct can_hw_mb *hwmb;
	int rx_mb, i;

	/* Set global mask to receive all messages */
	__raw_writel(0, flexcan->io_base + CAN_HW_REG_RXGMASK);
	__raw_writel(0, flexcan->io_base + CAN_HW_REG_RX14MASK);
	__raw_writel(0, flexcan->io_base + CAN_HW_REG_RX15MASK);

	memset(flexcan->hwmb, 0, sizeof(*hwmb) * FLEXCAN_MAX_MB);
	/* Set individual mask to receive all messages */
	memset(flexcan->rx_mask, 0, sizeof(unsigned int) * FLEXCAN_MAX_MB);

	if (flexcan->fifo)
		rx_mb = FLEXCAN_MAX_FIFO_MB;
	else
		rx_mb = flexcan->maxmb - flexcan->xmit_maxmb;

	hwmb = flexcan->hwmb;
	if (flexcan->fifo) {
		unsigned long *id_table = flexcan->io_base + CAN_FIFO_BASE;
		for (i = 0; i < rx_mb; i++)
			id_table[i] = 0;
	} else {
		for (i = 0; i < rx_mb; i++) {
			hwmb[i].mb_cs.cs.code = CAN_MB_RX_EMPTY;
			/*
			 * IDE bit can not control by mask registers
			 * So set message buffer to receive extend
			 * or standard message.
			 */
			if (flexcan->ext_msg && flexcan->std_msg)
				hwmb[i].mb_cs.cs.ide = i & 1;
			else {
				if (flexcan->ext_msg)
					hwmb[i].mb_cs.cs.ide = 1;
			}
		}
	}

	for (; i <= flexcan->maxmb; i++)
		hwmb[i].mb_cs.cs.code = CAN_MB_TX_INACTIVE;

	flexcan->xmit_mb = rx_mb;
}
