/*
 * CAN bus driver for Bosch M_CAN controller
 *
 * Copyright (C) 2014 Freescale Semiconductor, Inc.
 *	Dong Aisheng <b29396@freescale.com>
 *
 * Bosch M_CAN user manual can be obtained from:
 * http://www.bosch-semiconductors.de/media/pdf_1/ipmodules_1/m_can/
 * mcan_users_manual_v302.pdf
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/if_arp.h>
#include <linux/if_ether.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/netdevice.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/pinctrl/consumer.h>
#include <linux/platform_device.h>

#include <linux/can/dev.h>

/* napi related */
#define M_CAN_NAPI_WEIGHT	64

/* registers definition */
enum m_can_reg {
	M_CAN_CREL	= 0x0,
	M_CAN_ENDN	= 0x4,
	M_CAN_CUST	= 0x8,
	M_CAN_FBTP	= 0xc,
	M_CAN_TEST	= 0x10,
	M_CAN_RWD	= 0x14,
	M_CAN_CCCR	= 0x18,
	M_CAN_BTP	= 0x1c,
	M_CAN_TSCC	= 0x20,
	M_CAN_TSCV	= 0x24,
	M_CAN_TOCC	= 0x28,
	M_CAN_TOCV	= 0x2c,
	M_CAN_ECR	= 0x40,
	M_CAN_PSR	= 0x44,
	M_CAN_IR	= 0x50,
	M_CAN_IE	= 0x54,
	M_CAN_ILS	= 0x58,
	M_CAN_ILE	= 0x5c,
	M_CAN_GFC	= 0x80,
	M_CAN_SIDFC	= 0x84,
	M_CAN_XIDFC	= 0x88,
	M_CAN_XIDAM	= 0x90,
	M_CAN_HPMS	= 0x94,
	M_CAN_NDAT1	= 0x98,
	M_CAN_NDAT2	= 0x9c,
	M_CAN_RXF0C	= 0xa0,
	M_CAN_RXF0S	= 0xa4,
	M_CAN_RXF0A	= 0xa8,
	M_CAN_RXBC	= 0xac,
	M_CAN_RXF1C	= 0xb0,
	M_CAN_RXF1S	= 0xb4,
	M_CAN_RXF1A	= 0xb8,
	M_CAN_RXESC	= 0xbc,
	M_CAN_TXBC	= 0xc0,
	M_CAN_TXFQS	= 0xc4,
	M_CAN_TXESC	= 0xc8,
	M_CAN_TXBRP	= 0xcc,
	M_CAN_TXBAR	= 0xd0,
	M_CAN_TXBCR	= 0xd4,
	M_CAN_TXBTO	= 0xd8,
	M_CAN_TXBCF	= 0xdc,
	M_CAN_TXBTIE	= 0xe0,
	M_CAN_TXBCIE	= 0xe4,
	M_CAN_TXEFC	= 0xf0,
	M_CAN_TXEFS	= 0xf4,
	M_CAN_TXEFA	= 0xf8,
};

/* m_can lec values */
enum m_can_lec_type {
	LEC_NO_ERROR = 0,
	LEC_STUFF_ERROR,
	LEC_FORM_ERROR,
	LEC_ACK_ERROR,
	LEC_BIT1_ERROR,
	LEC_BIT0_ERROR,
	LEC_CRC_ERROR,
	LEC_UNUSED,
};
/* Test Register (TEST) */
#define TEST_LBCK	BIT(4)

/* CC Control Register(CCCR) */
#define CCCR_TEST	BIT(7)
#define CCCR_MON	BIT(5)
#define CCCR_CCE	BIT(1)
#define CCCR_INIT	BIT(0)

/* Bit Timing & Prescaler Register (BTP) */
#define BTR_BRP_MASK		0x3ff
#define BTR_BRP_SHIFT		16
#define BTR_TSEG1_SHIFT		8
#define BTR_TSEG1_MASK		(0x3f << BTR_TSEG1_SHIFT)
#define BTR_TSEG2_SHIFT		4
#define BTR_TSEG2_MASK		(0xf << BTR_TSEG2_SHIFT)
#define BTR_SJW_SHIFT		0
#define BTR_SJW_MASK		0xf

/* Error Counter Register(ECR) */
#define ECR_RP			BIT(15)
#define ECR_REC_SHIFT		8
#define ECR_REC_MASK		(0x7f << ECR_REC_SHIFT)
#define ECR_TEC_SHIFT		0
#define ECR_TEC_MASK		0xff

/* Protocol Status Register(PSR) */
#define PSR_BO		BIT(7)
#define PSR_EW		BIT(6)
#define PSR_EP		BIT(5)
#define PSR_LEC_MASK	0x7

/* Interrupt Register(IR) */
#define IR_ALL_INT	0xffffffff
#define IR_STE		BIT(31)
#define IR_FOE		BIT(30)
#define IR_ACKE		BIT(29)
#define IR_BE		BIT(28)
#define IR_CRCE		BIT(27)
#define IR_WDI		BIT(26)
#define IR_BO		BIT(25)
#define IR_EW		BIT(24)
#define IR_EP		BIT(23)
#define IR_ELO		BIT(22)
#define IR_BEU		BIT(21)
#define IR_BEC		BIT(20)
#define IR_DRX		BIT(19)
#define IR_TOO		BIT(18)
#define IR_MRAF		BIT(17)
#define IR_TSW		BIT(16)
#define IR_TEFL		BIT(15)
#define IR_TEFF		BIT(14)
#define IR_TEFW		BIT(13)
#define IR_TEFN		BIT(12)
#define IR_TFE		BIT(11)
#define IR_TCF		BIT(10)
#define IR_TC		BIT(9)
#define IR_HPM		BIT(8)
#define IR_RF1L		BIT(7)
#define IR_RF1F		BIT(6)
#define IR_RF1W		BIT(5)
#define IR_RF1N		BIT(4)
#define IR_RF0L		BIT(3)
#define IR_RF0F		BIT(2)
#define IR_RF0W		BIT(1)
#define IR_RF0N		BIT(0)
#define IR_ERR_STATE	(IR_BO | IR_EW | IR_EP)
#define IR_ERR_BUS	(IR_STE	| IR_FOE | IR_ACKE | IR_BE | IR_CRCE | \
		IR_WDI | IR_ELO | IR_BEU | IR_BEC | IR_TOO | IR_MRAF | \
		IR_TSW | IR_TEFL | IR_RF1L | IR_RF0L)
#define IR_ERR_ALL	(IR_ERR_STATE | IR_ERR_BUS)

/* Rx FIFO 0/1 Configuration (RXF0C/RXF1C) */
#define RXFC_FWM_OFF	24
#define RXFC_FWM_MASK	0x7f
#define RXFC_FWM_1	(1 << RXFC_FWM_OFF)
#define RXFC_FS_OFF	16
#define RXFC_FS_MASK	0x7f

/* Rx FIFO 0/1 Status (RXF0S/RXF1S) */
#define RXFS_RFL	BIT(25)
#define RXFS_FF		BIT(24)
#define RXFS_FPI_OFF	16
#define RXFS_FPI_MASK	0x3f0000
#define RXFS_FGI_OFF	8
#define RXFS_FGI_MASK	0x3f00
#define RXFS_FFL_MASK	0x7f

/* Tx Buffer Configuration(TXBC) */
#define TXBC_NDTB_OFF	16
#define TXBC_NDTB_MASK	0x3f

/* Tx Buffer Element Size Configuration(TXESC) */
#define TXESC_TBDS_8BYTES	0x0
/* Tx Buffer Element */
#define TX_BUF_XTD	BIT(30)
#define TX_BUF_RTR	BIT(29)

/* Rx Buffer Element Size Configuration(TXESC) */
#define M_CAN_RXESC_8BYTES	0x0
/* Tx Buffer Element */
#define RX_BUF_ESI	BIT(31)
#define RX_BUF_XTD	BIT(30)
#define RX_BUF_RTR	BIT(29)

/* Message RAM Configuration (in bytes) */
#define SIDF_ELEMENT_SIZE	4
#define XIDF_ELEMENT_SIZE	8
#define RXF0_ELEMENT_SIZE	16
#define RXF1_ELEMENT_SIZE	16
#define RXB_ELEMENT_SIZE	16
#define TXE_ELEMENT_SIZE	8
#define TXB_ELEMENT_SIZE	16

/* m_can private data structure */
struct m_can_priv {
	struct can_priv can;	/* must be the first member */
	struct napi_struct napi;
	struct net_device *dev;
	struct device *device;
	struct clk *clk;
	void __iomem *base;
	u32 irqstatus;

	/* message ram configuration */
	void __iomem *mram_base;
	u32 mram_off;
	u32 sidf_elems;
	u32 sidf_off;
	u32 xidf_elems;
	u32 xidf_off;
	u32 rxf0_elems;
	u32 rxf0_off;
	u32 rxf1_elems;
	u32 rxf1_off;
	u32 rxb_elems;
	u32 rxb_off;
	u32 txe_elems;
	u32 txe_off;
	u32 txb_elems;
	u32 txb_off;
};

static inline u32 m_can_read(const struct m_can_priv *priv, enum m_can_reg reg)
{
	return readl(priv->base + reg);
}

static inline void m_can_write(const struct m_can_priv *priv,
				enum m_can_reg reg, u32 val)
{
	writel(val, priv->base + reg);
}

static inline void m_can_config_endisable(const struct m_can_priv *priv,
				bool enable)
{
	u32 cccr = m_can_read(priv, M_CAN_CCCR);
	u32 timeout = 10, val;

	if (enable) {
		/* enable m_can configuration */
		m_can_write(priv, M_CAN_CCCR, cccr | CCCR_INIT);
		/* CCCR.CCE can only be set/reset while CCCR.INIT = '1' */
		m_can_write(priv, M_CAN_CCCR, cccr | CCCR_INIT | CCCR_CCE);
	} else {
		m_can_write(priv, M_CAN_CCCR, cccr & ~(CCCR_INIT | CCCR_CCE));
	}

	/* there's a delay for module initialization */
	val = enable ? CCCR_INIT | CCCR_CCE : 0;
	while ((m_can_read(priv, M_CAN_CCCR) & (CCCR_INIT | CCCR_CCE))
				!= val) {
		if (timeout == 0) {
			netdev_warn(priv->dev, "Failed to init module\n");
			return;
		}
		timeout--;
		udelay(1);
	}
}

static void m_can_enable_all_interrupts(struct m_can_priv *priv, bool enable)
{
	m_can_write(priv, M_CAN_ILE, enable ? 0x00000003 : 0x0);
}

static int m_can_do_rx_poll(struct net_device *dev, int quota)
{
	struct m_can_priv *priv = netdev_priv(dev);
	struct net_device_stats *stats = &dev->stats;
	struct sk_buff *skb;
	struct can_frame *frame;
	u32 rxfs, flags, fgi;
	u32 num_rx_pkts = 0;

	rxfs = m_can_read(priv, M_CAN_RXF0S);
	if (!(rxfs & RXFS_FFL_MASK)) {
		netdev_dbg(dev, "no messages in fifo0\n");
		return 0;
	}

	while ((rxfs & RXFS_FFL_MASK) && (quota > 0)) {
		netdev_dbg(dev, "fifo0 status 0x%x\n", rxfs);
		if (rxfs & RXFS_RFL)
			netdev_warn(dev, "Rx FIFO 0 Message Lost\n");

		skb = alloc_can_skb(dev, &frame);
		if (!skb) {
			stats->rx_dropped++;
			return -ENOMEM;
		}

		fgi = (rxfs & RXFS_FGI_MASK) >> RXFS_FGI_OFF;
		flags = readl(priv->mram_base + priv->rxf0_off + fgi * 16);
		if (flags & RX_BUF_XTD)
			frame->can_id = (flags & CAN_EFF_MASK) | CAN_EFF_FLAG;
		else
			frame->can_id = (flags >> 18) & CAN_SFF_MASK;
		netdev_dbg(dev, "R0 0x%x\n", flags);

		if (flags & RX_BUF_RTR) {
			frame->can_id |= CAN_RTR_FLAG;
		} else {
			flags = readl(priv->mram_base +
					priv->rxf0_off + fgi * 16 + 0x4);
			frame->can_dlc = get_can_dlc((flags >> 16) & 0x0F);
			netdev_dbg(dev, "R1 0x%x\n", flags);

			*(u32 *)(frame->data + 0) = readl(priv->mram_base +
					priv->rxf0_off + fgi * 16 + 0x8);
			*(u32 *)(frame->data + 4) = readl(priv->mram_base +
					priv->rxf0_off + fgi * 16 + 0xC);
			netdev_dbg(dev, "R2 0x%x\n", *(u32 *)(frame->data + 0));
			netdev_dbg(dev, "R3 0x%x\n", *(u32 *)(frame->data + 4));
		}

		/* acknowledge rx fifo 0 */
		m_can_write(priv, M_CAN_RXF0A, fgi);

		netif_receive_skb(skb);
		netdev_dbg(dev, "new packet received\n");

		stats->rx_packets++;
		stats->rx_bytes += frame->can_dlc;

		can_led_event(dev, CAN_LED_EVENT_RX);

		quota--;
		num_rx_pkts++;
		rxfs = m_can_read(priv, M_CAN_RXF0S);
	};

	return num_rx_pkts;
}

static int m_can_handle_lost_msg(struct net_device *dev)
{
	struct net_device_stats *stats = &dev->stats;
	struct sk_buff *skb;
	struct can_frame *frame;

	netdev_err(dev, "msg lost in rxf0\n");

	skb = alloc_can_err_skb(dev, &frame);
	if (unlikely(!skb))
		return 0;

	frame->can_id |= CAN_ERR_CRTL;
	frame->data[1] = CAN_ERR_CRTL_RX_OVERFLOW;
	stats->rx_errors++;
	stats->rx_over_errors++;

	netif_receive_skb(skb);

	return 1;
}

static int m_can_handle_bus_err(struct net_device *dev,
				enum m_can_lec_type lec_type)
{
	struct m_can_priv *priv = netdev_priv(dev);
	struct net_device_stats *stats = &dev->stats;
	struct can_frame *cf;
	struct sk_buff *skb;

	/* early exit if no lec update */
	if (lec_type == LEC_UNUSED)
		return 0;

	/* propagate the error condition to the CAN stack */
	skb = alloc_can_err_skb(dev, &cf);
	if (unlikely(!skb))
		return 0;

	/*
	 * check for 'last error code' which tells us the
	 * type of the last error to occur on the CAN bus
	 */
	priv->can.can_stats.bus_error++;
	stats->rx_errors++;
	cf->can_id |= CAN_ERR_PROT | CAN_ERR_BUSERROR;
	cf->data[2] |= CAN_ERR_PROT_UNSPEC;

	switch (lec_type) {
	case LEC_STUFF_ERROR:
		netdev_dbg(dev, "stuff error\n");
		cf->data[2] |= CAN_ERR_PROT_STUFF;
		break;
	case LEC_FORM_ERROR:
		netdev_dbg(dev, "form error\n");
		cf->data[2] |= CAN_ERR_PROT_FORM;
		break;
	case LEC_ACK_ERROR:
		netdev_dbg(dev, "ack error\n");
		cf->data[3] |= (CAN_ERR_PROT_LOC_ACK |
				CAN_ERR_PROT_LOC_ACK_DEL);
		break;
	case LEC_BIT1_ERROR:
		netdev_dbg(dev, "bit1 error\n");
		cf->data[2] |= CAN_ERR_PROT_BIT1;
		break;
	case LEC_BIT0_ERROR:
		netdev_dbg(dev, "bit0 error\n");
		cf->data[2] |= CAN_ERR_PROT_BIT0;
		break;
	case LEC_CRC_ERROR:
		netdev_dbg(dev, "CRC error\n");
		cf->data[3] |= (CAN_ERR_PROT_LOC_CRC_SEQ |
				CAN_ERR_PROT_LOC_CRC_DEL);
		break;
	default:
		break;
	}

	netif_receive_skb(skb);
	stats->rx_packets++;
	stats->rx_bytes += cf->can_dlc;

	return 1;
}

static int m_can_get_berr_counter(const struct net_device *dev,
				  struct can_berr_counter *bec)
{
	struct m_can_priv *priv = netdev_priv(dev);
	unsigned int ecr;

	ecr = m_can_read(priv, M_CAN_ECR);
	bec->rxerr = (ecr & ECR_REC_MASK) >> ECR_REC_SHIFT;
	bec->txerr = ecr & ECR_TEC_MASK;

	return 0;
}

static int m_can_handle_state_change(struct net_device *dev,
				enum can_state new_state)
{
	struct m_can_priv *priv = netdev_priv(dev);
	struct net_device_stats *stats = &dev->stats;
	struct can_frame *cf;
	struct sk_buff *skb;
	struct can_berr_counter bec;
	unsigned int ecr;

	/* propagate the error condition to the CAN stack */
	skb = alloc_can_err_skb(dev, &cf);
	if (unlikely(!skb))
		return 0;

	m_can_get_berr_counter(dev, &bec);

	switch (new_state) {
	case CAN_STATE_ERROR_ACTIVE:
		/* error warning state */
		priv->can.can_stats.error_warning++;
		priv->can.state = CAN_STATE_ERROR_WARNING;
		cf->can_id |= CAN_ERR_CRTL;
		cf->data[1] = (bec.txerr > bec.rxerr) ?
			CAN_ERR_CRTL_TX_WARNING :
			CAN_ERR_CRTL_RX_WARNING;
		cf->data[6] = bec.txerr;
		cf->data[7] = bec.rxerr;
		break;
	case CAN_STATE_ERROR_PASSIVE:
		/* error passive state */
		priv->can.can_stats.error_passive++;
		priv->can.state = CAN_STATE_ERROR_PASSIVE;
		cf->can_id |= CAN_ERR_CRTL;
		ecr = m_can_read(priv, M_CAN_ECR);
		if (ecr & ECR_RP)
			cf->data[1] |= CAN_ERR_CRTL_RX_PASSIVE;
		if (bec.txerr > 127)
			cf->data[1] |= CAN_ERR_CRTL_TX_PASSIVE;
		cf->data[6] = bec.txerr;
		cf->data[7] = bec.rxerr;
		break;
	case CAN_STATE_BUS_OFF:
		/* bus-off state */
		priv->can.state = CAN_STATE_BUS_OFF;
		cf->can_id |= CAN_ERR_BUSOFF;
		/*
		 * disable all interrupts in bus-off mode to ensure that
		 * the CPU is not hogged down
		 */
		m_can_enable_all_interrupts(priv, false);
		can_bus_off(dev);
		break;
	default:
		break;
	}

	netif_receive_skb(skb);
	stats->rx_packets++;
	stats->rx_bytes += cf->can_dlc;

	return 1;
}

static int m_can_poll(struct napi_struct *napi, int quota)
{
	struct net_device *dev = napi->dev;
	struct m_can_priv *priv = netdev_priv(dev);
	u32 work_done = 0;
	u32 irqstatus, psr;

	irqstatus = m_can_read(priv, M_CAN_IR);
	if (irqstatus)
		netdev_dbg(dev, "irqstatus updated! new 0x%x old 0x%x rxf0s 0x%x\n",
			irqstatus, priv->irqstatus,
			m_can_read(priv, M_CAN_RXF0S));

	irqstatus = irqstatus | priv->irqstatus;
	if (!irqstatus)
		goto end;

	psr = m_can_read(priv, M_CAN_PSR);
	if (irqstatus & IR_ERR_STATE) {
		if ((psr & PSR_EW) &&
			(priv->can.state != CAN_STATE_ERROR_WARNING)) {
			netdev_dbg(dev, "entered error warning state\n");
			work_done += m_can_handle_state_change(dev,
					CAN_STATE_ERROR_WARNING);
		}

		if ((psr & PSR_EP) &&
			(priv->can.state != CAN_STATE_ERROR_PASSIVE)) {
			netdev_dbg(dev, "entered error warning state\n");
			work_done += m_can_handle_state_change(dev,
					CAN_STATE_ERROR_PASSIVE);
		}

		if ((psr & PSR_BO) &&
			(priv->can.state != CAN_STATE_BUS_OFF)) {
			netdev_dbg(dev, "entered error warning state\n");
			work_done += m_can_handle_state_change(dev,
					CAN_STATE_BUS_OFF);
		}
	}

	if (irqstatus & IR_ERR_BUS) {
		if (irqstatus & IR_RF0L)
			work_done += m_can_handle_lost_msg(dev);

		/* handle lec errors on the bus */
		if (psr & LEC_UNUSED)
			work_done += m_can_handle_bus_err(dev,
					psr & LEC_UNUSED);

		/* other unproccessed error interrupts */
		if (irqstatus & IR_WDI)
			netdev_err(dev, "Message RAM Watchdog event due to missing READY\n");
		if (irqstatus & IR_TOO)
			netdev_err(dev, "Timeout reached\n");
		if (irqstatus & IR_MRAF)
			netdev_err(dev, "Message RAM access failure occurred\n");
	}

	if (irqstatus & IR_RF0N)
		/* handle events corresponding to receive message objects */
		work_done += m_can_do_rx_poll(dev, (quota - work_done));

	if (work_done < quota) {
		napi_complete(napi);
		/* enable all IRQs */
		m_can_enable_all_interrupts(priv, true);
	}

end:
	return work_done;
}

static irqreturn_t m_can_isr(int irq, void *dev_id)
{
	struct net_device *dev = (struct net_device *)dev_id;
	struct m_can_priv *priv = netdev_priv(dev);
	struct net_device_stats *stats = &dev->stats;
	u32 ir;

	ir = m_can_read(priv, M_CAN_IR);
	if (!ir)
		return IRQ_NONE;

	netdev_dbg(dev, "ir 0x%x rxfs0 0x%x\n", ir,
			m_can_read(priv, M_CAN_RXF0S));

	/* ACK all irqs */
	if (ir & IR_ALL_INT)
		m_can_write(priv, M_CAN_IR, ir);

	/*
	 * schedule NAPI in case of
	 * - rx IRQ
	 * - state change IRQ
	 * - bus error IRQ and bus error reporting
	 */
	if ((ir & IR_RF0N) || (ir & IR_ERR_ALL)) {
		priv->irqstatus = ir;
		m_can_enable_all_interrupts(priv, false);
		napi_schedule(&priv->napi);
	}

	/* transmission complete interrupt */
	if (ir & IR_TC) {
		netdev_dbg(dev, "tx complete\n");
		stats->tx_bytes += can_get_echo_skb(dev, 0);
		stats->tx_packets++;
		can_led_event(dev, CAN_LED_EVENT_TX);
		netif_wake_queue(dev);
	}

	return IRQ_HANDLED;
}

static const struct can_bittiming_const m_can_bittiming_const = {
	.name = KBUILD_MODNAME,
	.tseg1_min = 2,		/* Time segment 1 = prop_seg + phase_seg1 */
	.tseg1_max = 64,
	.tseg2_min = 1,		/* Time segment 2 = phase_seg2 */
	.tseg2_max = 16,
	.sjw_max = 16,
	.brp_min = 1,
	.brp_max = 1024,
	.brp_inc = 1,
};

static int m_can_set_bittiming(struct net_device *dev)
{
	struct m_can_priv *priv = netdev_priv(dev);
	const struct can_bittiming *bt = &priv->can.bittiming;
	u16 brp, sjw, tseg1, tseg2;
	u32 reg_btp;

	brp = bt->brp - 1;
	sjw = bt->sjw - 1;
	tseg1 = bt->prop_seg + bt->phase_seg1 - 1;
	tseg2 = bt->phase_seg2 - 1;
	reg_btp = (brp << BTR_BRP_SHIFT) | (sjw << BTR_SJW_SHIFT) |
			(tseg1 << BTR_TSEG1_SHIFT) | (tseg2 << BTR_TSEG2_SHIFT);
	m_can_write(priv, M_CAN_BTP, reg_btp);
	netdev_dbg(dev, "setting BTP 0x%x\n", reg_btp);

	return 0;
}

/*
 * Configure M_CAN chip:
 * - set rx buffer/fifo element size
 * - configure rx fifo
 * - accept non-matching frame into fifo 0
 * - configure tx buffer
 * - configure mode
 * - setup bittiming
 */
static void m_can_chip_config(struct net_device *dev)
{
	struct m_can_priv *priv = netdev_priv(dev);
	u32 cccr, test;

	m_can_config_endisable(priv, true);

	/* RX Buffer/FIFO Element Size 8 bytes data field */
	m_can_write(priv, M_CAN_RXESC, M_CAN_RXESC_8BYTES);

	/* Accept Non-matching Frames Into FIFO 0 */
	m_can_write(priv, M_CAN_GFC, 0x0);

	/* only support one Tx Buffer currently */
	m_can_write(priv, M_CAN_TXBC, (1 << TXBC_NDTB_OFF) | priv->txb_off);

	/* only support 8 bytes firstly */
	m_can_write(priv, M_CAN_TXESC, TXESC_TBDS_8BYTES);

	m_can_write(priv, M_CAN_TXEFC, 0x00010000 | priv->txe_off);

	/* rx fifo configuration, blocking mode, fifo size 1 */
	m_can_write(priv, M_CAN_RXF0C, (priv->rxf0_elems << RXFC_FS_OFF) |
		RXFC_FWM_1 | priv->rxf0_off);

	m_can_write(priv, M_CAN_RXF1C, (priv->rxf1_elems << RXFC_FS_OFF) |
		RXFC_FWM_1 | priv->rxf1_off);

	cccr = m_can_read(priv, M_CAN_CCCR);
	cccr &= ~(CCCR_TEST | CCCR_MON);
	test = m_can_read(priv, M_CAN_TEST);
	test &= ~TEST_LBCK;

	if (priv->can.ctrlmode & CAN_CTRLMODE_LISTENONLY)
		cccr |= CCCR_MON;

	if (priv->can.ctrlmode & CAN_CTRLMODE_LOOPBACK) {
		cccr |= CCCR_TEST;
		test |= TEST_LBCK;
	}

	m_can_write(priv, M_CAN_CCCR, cccr);
	m_can_write(priv, M_CAN_TEST, test);

	/* enable all interrupts */
	m_can_write(priv, M_CAN_IR, IR_ALL_INT);
	m_can_write(priv, M_CAN_IE, IR_ALL_INT);
	m_can_write(priv, M_CAN_ILS, 0xFFFF0000);

	/* set bittiming params */
	m_can_set_bittiming(dev);

	m_can_config_endisable(priv, false);
}

static void m_can_start(struct net_device *dev)
{
	struct m_can_priv *priv = netdev_priv(dev);

	/* basic m_can configuration */
	m_can_chip_config(dev);

	priv->can.state = CAN_STATE_ERROR_ACTIVE;

	/* enable status change, error and module interrupts */
	m_can_enable_all_interrupts(priv, true);
}

static int m_can_set_mode(struct net_device *dev, enum can_mode mode)
{
	switch (mode) {
	case CAN_MODE_START:
		m_can_start(dev);
		netif_wake_queue(dev);
		break;
	default:
		return -EOPNOTSUPP;
	}

	return 0;
}

static void free_m_can_dev(struct net_device *dev)
{
	free_candev(dev);
}

static struct net_device *alloc_m_can_dev(void)
{
	struct net_device *dev;
	struct m_can_priv *priv;

	dev = alloc_candev(sizeof(struct m_can_priv), 1);
	if (!dev)
		return NULL;

	priv = netdev_priv(dev);
	netif_napi_add(dev, &priv->napi, m_can_poll, M_CAN_NAPI_WEIGHT);

	priv->dev = dev;
	priv->can.bittiming_const = &m_can_bittiming_const;
	priv->can.do_set_mode = m_can_set_mode;
	priv->can.do_get_berr_counter = m_can_get_berr_counter;
	priv->can.ctrlmode_supported = CAN_CTRLMODE_LOOPBACK |
					CAN_CTRLMODE_LISTENONLY |
					CAN_CTRLMODE_BERR_REPORTING;

	return dev;
}


static int m_can_open(struct net_device *dev)
{
	int err;
	struct m_can_priv *priv = netdev_priv(dev);

	clk_prepare_enable(priv->clk);

	/* open the can device */
	err = open_candev(dev);
	if (err) {
		netdev_err(dev, "failed to open can device\n");
		goto exit_open_fail;
	}

	/* register interrupt handler */
	err = request_irq(dev->irq, m_can_isr, IRQF_SHARED, dev->name,
				dev);
	if (err < 0) {
		netdev_err(dev, "failed to request interrupt\n");
		goto exit_irq_fail;
	}

	/* start the m_can controller */
	m_can_start(dev);

	can_led_event(dev, CAN_LED_EVENT_OPEN);
	napi_enable(&priv->napi);
	netif_start_queue(dev);

	return 0;

exit_irq_fail:
	close_candev(dev);
exit_open_fail:
	clk_disable_unprepare(priv->clk);
	return err;
}

static void m_can_stop(struct net_device *dev)
{
	struct m_can_priv *priv = netdev_priv(dev);

	/* disable all interrupts */
	m_can_enable_all_interrupts(priv, false);

	/* set the state as STOPPED */
	priv->can.state = CAN_STATE_STOPPED;
}


static int m_can_close(struct net_device *dev)
{
	struct m_can_priv *priv = netdev_priv(dev);

	netif_stop_queue(dev);
	napi_disable(&priv->napi);
	m_can_stop(dev);
	free_irq(dev->irq, dev);
	close_candev(dev);
	can_led_event(dev, CAN_LED_EVENT_STOP);

	return 0;
}

static netdev_tx_t m_can_start_xmit(struct sk_buff *skb,
					struct net_device *dev)
{
	struct m_can_priv *priv = netdev_priv(dev);
	struct can_frame *frame = (struct can_frame *)skb->data;
	u32 flags = 0, id;

	if (can_dropped_invalid_skb(dev, skb))
		return NETDEV_TX_OK;

	netif_stop_queue(dev);

	if (frame->can_id & CAN_RTR_FLAG)
		flags |= TX_BUF_RTR;

	if (frame->can_id & CAN_EFF_FLAG) {
		id = frame->can_id & CAN_EFF_MASK;
		flags |= TX_BUF_XTD;
	} else {
		id = ((frame->can_id & CAN_SFF_MASK) << 18);
	}

	/* message ram configuration */
	writel(id | flags, priv->mram_base + priv->txb_off);
	writel(frame->can_dlc << 16, priv->mram_base + priv->txb_off + 0x4);
	writel(*(u32 *)(frame->data + 0),
		priv->mram_base + priv->txb_off + 0x8);
	writel(*(u32 *)(frame->data + 4),
		priv->mram_base + priv->txb_off + 0xc);

	can_put_echo_skb(skb, dev, 0);

	/* enable first TX buffer to start transfer  */
	m_can_write(priv, M_CAN_TXBTIE, 0x00000001);
	m_can_write(priv, M_CAN_TXBAR, 0x00000001);

	return NETDEV_TX_OK;
}

static const struct net_device_ops m_can_netdev_ops = {
	.ndo_open = m_can_open,
	.ndo_stop = m_can_close,
	.ndo_start_xmit = m_can_start_xmit,
};

static int register_m_can_dev(struct net_device *dev)
{
	dev->flags |= IFF_ECHO;	/* we support local echo */
	dev->netdev_ops = &m_can_netdev_ops;

	return register_candev(dev);
}

static const struct of_device_id m_can_of_table[] = {
	{ .compatible = "bosch,m_can", .data = NULL },
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, m_can_of_table);

static int m_can_of_parse_mram(struct platform_device *pdev,
				struct m_can_priv *priv)
{
	struct device_node *np = pdev->dev.of_node;
	struct resource *res;
	void __iomem *addr;
	u32 out_val[8];
	int ret;

	/* message ram could be shared */
	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "message_ram");
	if (!res)
		return -ENODEV;

	addr = devm_ioremap(&pdev->dev, res->start, resource_size(res));
	if (!addr)
		return -ENODEV;

	/* get message ram configuration */
	ret = of_property_read_u32_array(np, "mram-cfg",
				out_val, sizeof(out_val) / 4);
	if (ret) {
		dev_err(&pdev->dev, "can not get message ram configuration\n");
		return -ENODEV;
	}

	priv->mram_base = addr;
	priv->mram_off = out_val[0];
	priv->sidf_elems = out_val[1];
	priv->sidf_off = priv->mram_off;
	priv->xidf_elems = out_val[2];
	priv->xidf_off = priv->sidf_off + priv->sidf_elems * SIDF_ELEMENT_SIZE;
	priv->rxf0_elems = out_val[3] & RXFC_FS_MASK;
	priv->rxf0_off = priv->xidf_off + priv->xidf_elems * XIDF_ELEMENT_SIZE;
	priv->rxf1_elems = out_val[4] & RXFC_FS_MASK;
	priv->rxf1_off = priv->rxf0_off + priv->rxf0_elems * RXF0_ELEMENT_SIZE;
	priv->rxb_elems = out_val[5];
	priv->rxb_off = priv->rxf1_off + priv->rxf1_elems * RXF1_ELEMENT_SIZE;
	priv->txe_elems = out_val[6];
	priv->txe_off = priv->rxb_off + priv->rxb_elems * RXB_ELEMENT_SIZE;
	priv->txb_elems = out_val[7] & TXBC_NDTB_MASK;
	priv->txb_off = priv->txe_off + priv->txe_elems * TXE_ELEMENT_SIZE;

	dev_dbg(&pdev->dev, "mram_base =%p mram_off =0x%x "
		"sidf %d xidf %d rxf0 %d rxf1 %d rxb %d txe %d txb %d\n",
		priv->mram_base, priv->mram_off, priv->sidf_elems,
		priv->xidf_elems, priv->rxf0_elems, priv->rxf1_elems,
		priv->rxb_elems, priv->txe_elems, priv->txb_elems);

	return 0;
}

static int m_can_plat_probe(struct platform_device *pdev)
{
	struct net_device *dev;
	struct m_can_priv *priv;
	struct pinctrl *pinctrl;
	struct resource *res;
	void __iomem *addr;
	struct clk *clk;
	int irq, ret;

	pinctrl = devm_pinctrl_get_select_default(&pdev->dev);
	if (IS_ERR(pinctrl))
		dev_warn(&pdev->dev,
			"failed to configure pins from driver\n");

	clk = devm_clk_get(&pdev->dev, NULL);
	if (IS_ERR(clk)) {
		dev_err(&pdev->dev, "no clock find\n");
		return PTR_ERR(clk);
	}

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "canfd");
	addr = devm_request_and_ioremap(&pdev->dev, res);
	irq = platform_get_irq(pdev, 0);
	if (!addr || irq < 0)
		return -EINVAL;

	/* allocate the m_can device */
	dev = alloc_m_can_dev();
	if (!dev)
		return -ENOMEM;

	priv = netdev_priv(dev);
	dev->irq = irq;
	priv->base = addr;
	priv->device = &pdev->dev;
	priv->clk = clk;
	priv->can.clock.freq = clk_get_rate(clk);

	ret = m_can_of_parse_mram(pdev, priv);
	if (ret)
		goto failed_free_dev;

	platform_set_drvdata(pdev, dev);
	SET_NETDEV_DEV(dev, &pdev->dev);

	ret = register_m_can_dev(dev);
	if (ret) {
		dev_err(&pdev->dev, "registering %s failed (err=%d)\n",
			KBUILD_MODNAME, ret);
		goto failed_free_dev;
	}

	devm_can_led_init(dev);

	dev_info(&pdev->dev, "%s device registered (regs=%p, irq=%d)\n",
		 KBUILD_MODNAME, priv->base, dev->irq);

	return 0;

failed_free_dev:
	free_m_can_dev(dev);
	return ret;
}

#ifdef CONFIG_PM
static int m_can_suspend(struct device *dev)
{
	struct net_device *ndev = dev_get_drvdata(dev);
	struct m_can_priv *priv = netdev_priv(ndev);

	if (netif_running(ndev)) {
		netif_stop_queue(ndev);
		netif_device_detach(ndev);
	}

	/* TODO: enter low power */

	priv->can.state = CAN_STATE_SLEEPING;

	return 0;
}

static int m_can_resume(struct device *dev)
{
	struct net_device *ndev = dev_get_drvdata(dev);
	struct m_can_priv *priv = netdev_priv(ndev);

	/* TODO: exit low power */

	priv->can.state = CAN_STATE_ERROR_ACTIVE;

	if (netif_running(ndev)) {
		netif_device_attach(ndev);
		netif_start_queue(ndev);
	}

	return 0;
}
#endif

static void unregister_m_can_dev(struct net_device *dev)
{
	unregister_candev(dev);
}

static int m_can_plat_remove(struct platform_device *pdev)
{
	struct net_device *dev = platform_get_drvdata(pdev);

	unregister_m_can_dev(dev);
	platform_set_drvdata(pdev, NULL);

	free_m_can_dev(dev);

	return 0;
}

static const struct dev_pm_ops m_can_pmops = {
	SET_SYSTEM_SLEEP_PM_OPS(m_can_suspend, m_can_resume)
};

static struct platform_driver m_can_plat_driver = {
	.driver = {
		.name = KBUILD_MODNAME,
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(m_can_of_table),
		.pm     = &m_can_pmops,
	},
	.probe = m_can_plat_probe,
	.remove = m_can_plat_remove,
};

module_platform_driver(m_can_plat_driver);

MODULE_AUTHOR("Dong Aisheng <b29396@freescale.com>");
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("CAN bus driver for Bosch M_CAN controller");
