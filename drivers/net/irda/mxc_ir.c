/*
 * Copyright 2004-2010 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * Based on sa1100_ir.c - Copyright 2000-2001 Russell King
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
 * @file mxc_ir.c
 *
 * @brief Driver for the Freescale Semiconductor MXC FIRI.
 *
 * This driver is based on drivers/net/irda/sa1100_ir.c, by Russell King.
 *
 * @ingroup FIRI
 */

/*
 * Include Files
 */

#include <linux/slab.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/netdevice.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <linux/clk.h>

#include <net/irda/irda.h>
#include <net/irda/wrapper.h>
#include <net/irda/irda_device.h>

#include <asm/irq.h>
#include <asm/dma.h>
#include <mach/mxc_uart.h>
#include "mxc_ir.h"

#define IS_SIR(mi)		((mi)->speed <= 115200)
#define IS_MIR(mi)		((mi)->speed < 4000000 && (mi)->speed >= 576000)
#define IS_FIR(mi)		((mi)->speed >= 4000000)

#define SDMA_START_DELAY()	{  \
				volatile int j, k;\
				int i;\
				for (i = 0; i < 10000; i++)\
				k = j;\
				}

#define IRDA_FRAME_SIZE_LIMIT   2047
#define UART_BUFF_SIZE 		14384

#define UART4_UFCR_TXTL  	16
#define UART4_UFCR_RXTL 	1

#define FIRI_SDMA_TX
#define FIRI_SDMA_RX

/*!
 * This structure is a way for the low level driver to define their own
 * \b mxc_irda structure. This structure includes SK buffers, DMA buffers.
 * and has other elements that are specifically required by this driver.
 */
struct mxc_irda {
	/*!
	 * This keeps track of device is running or not
	 */
	unsigned char open;

	/*!
	 * This holds current FIRI communication speed
	 */
	int speed;

	/*!
	 * This holds FIRI communication speed for next packet
	 */
	int newspeed;

	/*!
	 * SK buffer for transmitter
	 */
	struct sk_buff *txskb;

	/*!
	 * SK buffer for receiver
	 */
	struct sk_buff *rxskb;

#ifdef FIRI_SDMA_RX
	/*!
	 * SK buffer for tasklet
	 */
	struct sk_buff *tskb;
#endif

	/*!
	 * DMA address for transmitter
	 */
	dma_addr_t dma_rx_buff_phy;

	/*!
	 * DMA address for receiver
	 */
	dma_addr_t dma_tx_buff_phy;

	/*!
	 * DMA Transmit buffer length
	 */
	unsigned int dma_tx_buff_len;

	/*!
	 * DMA channel for transmitter
	 */
	int txdma_ch;

	/*!
	 * DMA channel for receiver
	 */
	int rxdma_ch;

	/*!
	 * IrDA network device statistics
	 */
	struct net_device_stats stats;

	/*!
	 * The device structure used to get FIRI information
	 */
	struct device *dev;

	/*!
	 * Resource structure for UART, which will maintain base addresses and IRQs.
	 */
	struct resource *uart_res;

	/*!
	 * Base address of UART, used in readl and writel.
	 */
	void *uart_base;

	/*!
	 * Resource structure for FIRI, which will maintain base addresses and IRQs.
	 */
	struct resource *firi_res;

	/*!
	 * Base address of FIRI, used in readl and writel.
	 */
	void *firi_base;

	/*!
	 * UART IRQ number.
	 */
	int uart_irq;

	/*!
	 * Second UART IRQ number in case the interrupt lines are not muxed.
	 */
	int uart_irq1;

	/*!
	 * UART clock needed for baud rate calculations
	 */
	struct clk *uart_clk;

	/*!
	 * UART clock needed for baud rate calculations
	 */
	unsigned long uart_clk_rate;

	/*!
	 * FIRI clock needed for baud rate calculations
	 */
	struct clk *firi_clk;

	/*!
	 * FIRI IRQ number.
	 */
	int firi_irq;

	/*!
	 * IrLAP layer instance
	 */
	struct irlap_cb *irlap;

	/*!
	 * Driver supported baudrate capabilities
	 */
	struct qos_info qos;

	/*!
	 * Temporary transmit buffer used by the driver
	 */
	iobuff_t tx_buff;

	/*!
	 * Temporary receive buffer used by the driver
	 */
	iobuff_t rx_buff;

	/*!
	 * Pointer to platform specific data structure.
	 */
	struct mxc_ir_platform_data *mxc_ir_plat;

	/*!
	 * This holds the power management status of this module.
	 */
	int suspend;

};

extern void gpio_firi_active(void *, unsigned int);
extern void gpio_firi_inactive(void);
extern void gpio_firi_init(void);

void mxc_irda_firi_init(struct mxc_irda *si);
#ifdef FIRI_SDMA_RX
static void mxc_irda_fir_dma_rx_irq(void *id, int error_status,
				    unsigned int count);
#endif
#ifdef FIRI_SDMA_TX
static void mxc_irda_fir_dma_tx_irq(void *id, int error_status,
				    unsigned int count);
#endif

/*!
 * This function allocates and maps the receive buffer,
 * unless it is already allocated.
 *
 * @param   si   FIRI device specific structure.
 * @return  The function returns 0 on success and a non-zero value on
 *          failure.
 */
static int mxc_irda_rx_alloc(struct mxc_irda *si)
{
#ifdef FIRI_SDMA_RX
	mxc_dma_requestbuf_t dma_request;
#endif
	if (si->rxskb) {
		return 0;
	}

	si->rxskb = alloc_skb(IRDA_FRAME_SIZE_LIMIT + 1, GFP_ATOMIC);

	if (!si->rxskb) {
		dev_err(si->dev, "mxc_ir: out of memory for RX SKB\n");
		return -ENOMEM;
	}

	/*
	 * Align any IP headers that may be contained
	 * within the frame.
	 */
	skb_reserve(si->rxskb, 1);

#ifdef FIRI_SDMA_RX
	si->dma_rx_buff_phy =
	    dma_map_single(si->dev, si->rxskb->data, IRDA_FRAME_SIZE_LIMIT,
			   DMA_FROM_DEVICE);

	dma_request.num_of_bytes = IRDA_FRAME_SIZE_LIMIT;
	dma_request.dst_addr = si->dma_rx_buff_phy;
	dma_request.src_addr = si->firi_res->start;

	mxc_dma_config(si->rxdma_ch, &dma_request, 1, MXC_DMA_MODE_READ);
#endif
	return 0;
}

/*!
 * This function is called to disable the FIRI dma
 *
 * @param   si  FIRI port specific structure.
 */
static void mxc_irda_disabledma(struct mxc_irda *si)
{
	/* Stop all DMA activity. */
#ifdef FIRI_SDMA_TX
	mxc_dma_disable(si->txdma_ch);
#endif
#ifdef FIRI_SDMA_RX
	mxc_dma_disable(si->rxdma_ch);
#endif
}

/*!
 * This function is called to set the IrDA communications speed.
 *
 * @param   si     FIRI specific structure.
 * @param   speed  new Speed to be configured for.
 *
 * @return  The function returns 0 on success and a non-zero value on
 *          failure.
 */
static int mxc_irda_set_speed(struct mxc_irda *si, int speed)
{
	unsigned long flags;
	int ret = 0;
	unsigned int num, denom, baud;
	unsigned int cr;

	dev_dbg(si->dev, "speed:%d\n", speed);
	switch (speed) {
	case 9600:
	case 19200:
	case 38400:
	case 57600:
	case 115200:
		dev_dbg(si->dev, "starting SIR\n");
		baud = speed;
		if (IS_FIR(si)) {
#ifdef FIRI_SDMA_RX
			mxc_dma_disable(si->rxdma_ch);
#endif
			cr = readl(si->firi_base + FIRITCR);
			cr &= ~FIRITCR_TE;
			writel(cr, si->firi_base + FIRITCR);

			cr = readl(si->firi_base + FIRIRCR);
			cr &= ~FIRIRCR_RE;
			writel(cr, si->firi_base + FIRIRCR);

		}
		local_irq_save(flags);

		/* Disable Tx and Rx */
		cr = readl(si->uart_base + MXC_UARTUCR2);
		cr &= ~(MXC_UARTUCR2_RXEN | MXC_UARTUCR2_TXEN);
		writel(cr, si->uart_base + MXC_UARTUCR2);

		gpio_firi_inactive();

		num = baud / 100 - 1;
		denom = si->uart_clk_rate / 1600 - 1;
		if ((denom < 65536) && (si->uart_clk_rate > 1600)) {
			writel(num, si->uart_base + MXC_UARTUBIR);
			writel(denom, si->uart_base + MXC_UARTUBMR);
		}

		si->speed = speed;

		writel(0xFFFF, si->uart_base + MXC_UARTUSR1);
		writel(0xFFFF, si->uart_base + MXC_UARTUSR2);

		/* Enable Receive Overrun and Data Ready interrupts. */
		cr = readl(si->uart_base + MXC_UARTUCR4);
		cr |= (MXC_UARTUCR4_OREN | MXC_UARTUCR4_DREN);
		writel(cr, si->uart_base + MXC_UARTUCR4);

		cr = readl(si->uart_base + MXC_UARTUCR2);
		cr |= (MXC_UARTUCR2_RXEN | MXC_UARTUCR2_TXEN);
		writel(cr, si->uart_base + MXC_UARTUCR2);

		local_irq_restore(flags);
		break;
	case 4000000:
		local_irq_save(flags);

		/* Disable Receive Overrun and Data Ready interrupts. */
		cr = readl(si->uart_base + MXC_UARTUCR4);
		cr &= ~(MXC_UARTUCR4_OREN | MXC_UARTUCR4_DREN);
		writel(cr, si->uart_base + MXC_UARTUCR4);

		/* Disable Tx and Rx */
		cr = readl(si->uart_base + MXC_UARTUCR2);
		cr &= ~(MXC_UARTUCR2_RXEN | MXC_UARTUCR2_TXEN);
		writel(cr, si->uart_base + MXC_UARTUCR2);

		/*
		 * FIR configuration
		 */
		mxc_irda_disabledma(si);

		cr = readl(si->firi_base + FIRITCR);
		cr &= ~FIRITCR_TE;
		writel(cr, si->firi_base + FIRITCR);

		gpio_firi_active(si->firi_base + FIRITCR, FIRITCR_TPP);

		si->speed = speed;

		cr = readl(si->firi_base + FIRIRCR);
		cr |= FIRIRCR_RE;
		writel(cr, si->firi_base + FIRIRCR);

		dev_dbg(si->dev, "Going for fast IRDA ...\n");
		ret = mxc_irda_rx_alloc(si);

		/* clear RX status register */
		writel(0xFFFF, si->firi_base + FIRIRSR);
#ifdef FIRI_SDMA_RX
		if (si->rxskb) {
			mxc_dma_enable(si->rxdma_ch);
		}
#endif
		local_irq_restore(flags);

		break;
	default:
		dev_err(si->dev, "speed not supported by FIRI\n");
		break;
	}

	return ret;
}

/*!
 * This function is called to set the IrDA communications speed.
 *
 * @param   si     FIRI specific structure.
 *
 * @return  The function returns 0 on success and a non-zero value on
 *          failure.
 */
static inline int mxc_irda_fir_error(struct mxc_irda *si)
{
	struct sk_buff *skb = si->rxskb;
	unsigned int dd_error, crc_error, overrun_error;
	unsigned int sr;

	if (!skb) {
		dev_err(si->dev, "no skb!\n");
		return -1;
	}

	sr = readl(si->firi_base + FIRIRSR);
	dd_error = sr & FIRIRSR_DDE;
	crc_error = sr & FIRIRSR_CRCE;
	overrun_error = sr & FIRIRSR_RFO;

	if (!(dd_error | crc_error | overrun_error)) {
		return 0;
	}
	dev_err(si->dev, "dde,crce,rfo=%d,%d,%d.\n", dd_error, crc_error,
		overrun_error);
	si->stats.rx_errors++;
	if (crc_error) {
		si->stats.rx_crc_errors++;
	}
	if (dd_error) {
		si->stats.rx_frame_errors++;
	}
	if (overrun_error) {
		si->stats.rx_frame_errors++;
	}
	writel(sr, si->firi_base + FIRIRSR);

	return -1;
}

#ifndef FIRI_SDMA_RX
/*!
 * FIR interrupt service routine to handle receive.
 *
 * @param   dev     pointer to the net_device structure
 */
void mxc_irda_fir_irq_rx(struct net_device *dev)
{
	struct mxc_irda *si = dev->priv;
	struct sk_buff *skb = si->rxskb;
	unsigned int sr, len;
	int i;
	unsigned char *p = skb->data;

	/*
	 * Deal with any receive errors.
	 */
	if (mxc_irda_fir_error(si) != 0) {
		return;
	}

	sr = readl(si->firi_base + FIRIRSR);

	if (!(sr & FIRIRSR_RPE)) {
		return;
	}

	/*
	 * Coming here indicates that fir rx packet has been successfully recieved.
	 * And No error happened so far.
	 */
	writel(sr | FIRIRSR_RPE, si->firi_base + FIRIRSR);

	len = (sr & FIRIRSR_RFP) >> 8;

	/* 4 bytes of CRC */
	len -= 4;

	skb_put(skb, len);

	for (i = 0; i < len; i++) {
		*p++ = readb(si->firi_base + FIRIRXFIFO);
	}

	/* Discard the four CRC bytes */
	for (i = 0; i < 4; i++) {
		readb(si->firi_base + FIRIRXFIFO);
	}

	/*
	 * Deal with the case of packet complete.
	 */
	skb->dev = dev;
	skb->mac.raw = skb->data;
	skb->protocol = htons(ETH_P_IRDA);
	si->stats.rx_packets++;
	si->stats.rx_bytes += len;
	netif_rx(skb);

	si->rxskb = NULL;
	mxc_irda_rx_alloc(si);

	writel(0xFFFF, si->firi_base + FIRIRSR);

}
#endif

/*!
 * FIR interrupt service routine to handle transmit.
 *
 * @param   dev     pointer to the net_device structure
 */
void mxc_irda_fir_irq_tx(struct net_device *dev)
{
	struct mxc_irda *si = netdev_priv(dev);
	struct sk_buff *skb = si->txskb;
	unsigned int cr, sr;

	sr = readl(si->firi_base + FIRITSR);
	writel(sr, si->firi_base + FIRITSR);

	if (sr & FIRITSR_TC) {

#ifdef FIRI_SDMA_TX
		mxc_dma_disable(si->txdma_ch);
#endif
		cr = readl(si->firi_base + FIRITCR);
		cr &= ~(FIRITCR_TCIE | FIRITCR_TE);
		writel(cr, si->firi_base + FIRITCR);

		if (si->newspeed) {
			mxc_irda_set_speed(si, si->newspeed);
			si->newspeed = 0;
		}
		si->txskb = NULL;

		cr = readl(si->firi_base + FIRIRCR);
		cr |= FIRIRCR_RE;
		writel(cr, si->firi_base + FIRIRCR);

		writel(0xFFFF, si->firi_base + FIRIRSR);
		/*
		 * Account and free the packet.
		 */
		if (skb) {
#ifdef FIRI_SDMA_TX
			dma_unmap_single(si->dev, si->dma_tx_buff_phy, skb->len,
					 DMA_TO_DEVICE);
#endif
			si->stats.tx_packets++;
			si->stats.tx_bytes += skb->len;
			dev_kfree_skb_irq(skb);
		}
		/*
		 * Make sure that the TX queue is available for sending
		 * (for retries).  TX has priority over RX at all times.
		 */
		netif_wake_queue(dev);
	}
}

/*!
 * This is FIRI interrupt handler.
 *
 * @param   dev     pointer to the net_device structure
 */
void mxc_irda_fir_irq(struct net_device *dev)
{
	struct mxc_irda *si = netdev_priv(dev);
	unsigned int sr1, sr2;

	sr1 = readl(si->firi_base + FIRIRSR);
	sr2 = readl(si->firi_base + FIRITSR);

	if (sr2 & FIRITSR_TC)
		mxc_irda_fir_irq_tx(dev);
#ifndef FIRI_SDMA_RX
	if (sr1 & (FIRIRSR_RPE | FIRIRSR_RFO))
		mxc_irda_fir_irq_rx(dev);
#endif

}

/*!
 * This is the SIR transmit routine.
 *
 * @param   si     FIRI specific structure.
 *
 * @param   dev     pointer to the net_device structure
 *
 * @return  The function returns 0 on success and a non-zero value on
 *          failure.
 */
static int mxc_irda_sir_txirq(struct mxc_irda *si, struct net_device *dev)
{
	unsigned int sr1, sr2, cr;
	unsigned int status;

	sr1 = readl(si->uart_base + MXC_UARTUSR1);
	sr2 = readl(si->uart_base + MXC_UARTUSR2);
	cr = readl(si->uart_base + MXC_UARTUCR2);

	/*
	 * Echo cancellation for IRDA Transmit chars
	 * Disable the receiver and enable Transmit complete.
	 */
	cr &= ~MXC_UARTUCR2_RXEN;
	writel(cr, si->uart_base + MXC_UARTUCR2);
	cr = readl(si->uart_base + MXC_UARTUCR4);
	cr |= MXC_UARTUCR4_TCEN;
	writel(cr, si->uart_base + MXC_UARTUCR4);

	while ((sr1 & MXC_UARTUSR1_TRDY) && si->tx_buff.len) {

		writel(*si->tx_buff.data++, si->uart_base + MXC_UARTUTXD);
		si->tx_buff.len -= 1;
		sr1 = readl(si->uart_base + MXC_UARTUSR1);
	}

	if (si->tx_buff.len == 0) {
		si->stats.tx_packets++;
		si->stats.tx_bytes += si->tx_buff.data - si->tx_buff.head;

		/*Yoohoo...we are done...Lets stop Tx */
		cr = readl(si->uart_base + MXC_UARTUCR1);
		cr &= ~MXC_UARTUCR1_TRDYEN;
		writel(cr, si->uart_base + MXC_UARTUCR1);

		do {
			status = readl(si->uart_base + MXC_UARTUSR2);
		} while (!(status & MXC_UARTUSR2_TXDC));

		if (si->newspeed) {
			mxc_irda_set_speed(si, si->newspeed);
			si->newspeed = 0;
		}
		/* I'm hungry! */
		netif_wake_queue(dev);

		/* Is the transmit complete to reenable the receiver? */
		if (status & MXC_UARTUSR2_TXDC) {

			cr = readl(si->uart_base + MXC_UARTUCR2);
			cr |= MXC_UARTUCR2_RXEN;
			writel(cr, si->uart_base + MXC_UARTUCR2);
			/* Disable the Transmit complete interrupt bit */
			cr = readl(si->uart_base + MXC_UARTUCR4);
			cr &= ~MXC_UARTUCR4_TCEN;
			writel(cr, si->uart_base + MXC_UARTUCR4);
		}
	}

	return 0;
}

/*!
 * This is the SIR receive routine.
 *
 * @param   si     FIRI specific structure.
 *
 * @param   dev     pointer to the net_device structure
 *
 * @return  The function returns 0 on success and a non-zero value on
 *          failure.
 */
static int mxc_irda_sir_rxirq(struct mxc_irda *si, struct net_device *dev)
{
	unsigned int data, status;
	volatile unsigned int sr2;

	sr2 = readl(si->uart_base + MXC_UARTUSR2);
	while ((sr2 & MXC_UARTUSR2_RDR) == 1) {
		data = readl(si->uart_base + MXC_UARTURXD);
		status = data & 0xf400;
		if (status & MXC_UARTURXD_ERR) {
			dev_err(si->dev, "Receive an incorrect data =0x%x.\n",
				data);
			si->stats.rx_errors++;
			if (status & MXC_UARTURXD_OVRRUN) {
				si->stats.rx_fifo_errors++;
				dev_err(si->dev, "Rx overrun.\n");
			}
			if (status & MXC_UARTURXD_FRMERR) {
				si->stats.rx_frame_errors++;
				dev_err(si->dev, "Rx frame error.\n");
			}
			if (status & MXC_UARTURXD_PRERR) {
				dev_err(si->dev, "Rx parity error.\n");
			}
			/* Other: it is the Break char.
			 * Do nothing for it. throw out the data.
			 */
			async_unwrap_char(dev, &si->stats, &si->rx_buff,
					  (data & 0xFF));
		} else {
			/* It is correct data. */
			data &= 0xFF;
			async_unwrap_char(dev, &si->stats, &si->rx_buff, data);

			dev->last_rx = jiffies;
		}
		sr2 = readl(si->uart_base + MXC_UARTUSR2);

		writel(0xFFFF, si->uart_base + MXC_UARTUSR1);
		writel(0xFFFF, si->uart_base + MXC_UARTUSR2);
	}			/*while */
	return 0;

}

static irqreturn_t mxc_irda_irq(int irq, void *dev_id)
{
	struct net_device *dev = dev_id;
	struct mxc_irda *si = netdev_priv(dev);

	if (IS_FIR(si)) {
		mxc_irda_fir_irq(dev);
		return IRQ_HANDLED;
	}

	if (readl(si->uart_base + MXC_UARTUCR2) & MXC_UARTUCR2_RXEN) {
		mxc_irda_sir_rxirq(si, dev);
	}
	if ((readl(si->uart_base + MXC_UARTUCR1) & MXC_UARTUCR1_TRDYEN) &&
	    (readl(si->uart_base + MXC_UARTUSR1) & MXC_UARTUSR1_TRDY)) {
		mxc_irda_sir_txirq(si, dev);
	}

	return IRQ_HANDLED;
}

static irqreturn_t mxc_irda_tx_irq(int irq, void *dev_id)
{

	struct net_device *dev = dev_id;
	struct mxc_irda *si = netdev_priv(dev);

	mxc_irda_sir_txirq(si, dev);

	return IRQ_HANDLED;
}

static irqreturn_t mxc_irda_rx_irq(int irq, void *dev_id)
{

	struct net_device *dev = dev_id;
	struct mxc_irda *si = netdev_priv(dev);

	/* Clear the aging timer bit */
	writel(MXC_UARTUSR1_AGTIM, si->uart_base + MXC_UARTUSR1);

	mxc_irda_sir_rxirq(si, dev);

	return IRQ_HANDLED;
}

#ifdef FIRI_SDMA_RX
struct tasklet_struct dma_rx_tasklet;

static void mxc_irda_rx_task(unsigned long tparam)
{
	struct mxc_irda *si = (struct mxc_irda *)tparam;
	struct sk_buff *lskb = si->tskb;

	si->tskb = NULL;
	if (lskb) {
		lskb->mac_header = lskb->data;
		lskb->protocol = htons(ETH_P_IRDA);
		netif_rx(lskb);
	}
}

/*!
 * Receiver DMA callback routine.
 *
 * @param   id   pointer to network device structure
 * @param   error_status   used to pass error status to this callback function
 * @param   count   number of bytes received
 */
static void mxc_irda_fir_dma_rx_irq(void *id, int error_status,
				    unsigned int count)
{
	struct net_device *dev = id;
	struct mxc_irda *si = netdev_priv(dev);
	struct sk_buff *skb = si->rxskb;
	unsigned int cr;
	unsigned int len;

	cr = readl(si->firi_base + FIRIRCR);
	cr &= ~FIRIRCR_RE;
	writel(cr, si->firi_base + FIRIRCR);
	cr = readl(si->firi_base + FIRIRCR);
	cr |= FIRIRCR_RE;
	writel(cr, si->firi_base + FIRIRCR);
	len = count - 4;	/* remove 4 bytes for CRC */
	skb_put(skb, len);
	skb->dev = dev;
	si->tskb = skb;
	tasklet_schedule(&dma_rx_tasklet);

	if (si->dma_rx_buff_phy != 0)
		dma_unmap_single(si->dev, si->dma_rx_buff_phy,
				 IRDA_FRAME_SIZE_LIMIT, DMA_FROM_DEVICE);

	si->rxskb = NULL;
	mxc_irda_rx_alloc(si);

	SDMA_START_DELAY();
	writel(0xFFFF, si->firi_base + FIRIRSR);

	if (si->rxskb) {
		mxc_dma_enable(si->rxdma_ch);
	}
}
#endif

#ifdef FIRI_SDMA_TX
/*!
 * This function is called by SDMA Interrupt Service Routine to indicate
 * requested DMA transfer is completed.
 *
 * @param   id   pointer to network device structure
 * @param   error_status   used to pass error status to this callback function
 * @param   count   number of bytes sent
 */
static void mxc_irda_fir_dma_tx_irq(void *id, int error_status,
				    unsigned int count)
{
	struct net_device *dev = id;
	struct mxc_irda *si = netdev_priv(dev);

	mxc_dma_disable(si->txdma_ch);
}
#endif

/*!
 * This function is called by Linux IrDA network subsystem to
 * transmit the Infrared data packet. The TX DMA channel is configured
 * to transfer SK buffer data to FIRI TX FIFO along with DMA transfer
 * completion routine.
 *
 * @param   skb   The packet that is queued to be sent
 * @param   dev   net_device structure.
 *
 * @return  The function returns 0 on success and a negative value on
 *          failure.
 */
static int mxc_irda_hard_xmit(struct sk_buff *skb, struct net_device *dev)
{
	struct mxc_irda *si = netdev_priv(dev);
	int speed = irda_get_next_speed(skb);
	unsigned int cr;

	/*
	 * Does this packet contain a request to change the interface
	 * speed?  If so, remember it until we complete the transmission
	 * of this frame.
	 */
	if (speed != si->speed && speed != -1) {
		si->newspeed = speed;
	}

	/* If this is an empty frame, we can bypass a lot. */
	if (skb->len == 0) {
		if (si->newspeed) {
			si->newspeed = 0;
			mxc_irda_set_speed(si, speed);
		}
		dev_kfree_skb(skb);
		return 0;
	}

	/* We must not be transmitting... */
	netif_stop_queue(dev);
	if (IS_SIR(si)) {

		si->tx_buff.data = si->tx_buff.head;
		si->tx_buff.len = async_wrap_skb(skb, si->tx_buff.data,
						 si->tx_buff.truesize);
		cr = readl(si->uart_base + MXC_UARTUCR1);
		cr |= MXC_UARTUCR1_TRDYEN;
		writel(cr, si->uart_base + MXC_UARTUCR1);
		dev_kfree_skb(skb);
	} else {
		unsigned int mtt = irda_get_mtt(skb);
		unsigned char *p = skb->data;
		unsigned int skb_len = skb->len;
#ifdef FIRI_SDMA_TX
		mxc_dma_requestbuf_t dma_request;
#else
		unsigned int i, sr;
#endif

		skb_len = skb_len + ((4 - (skb_len % 4)) % 4);

		if (si->txskb) {
			BUG();
		}
		si->txskb = skb;

		/*
		 * If we have a mean turn-around time, impose the specified
		 * specified delay.  We could shorten this by timing from
		 * the point we received the packet.
		 */
		if (mtt) {
			udelay(mtt);
		}

		cr = readl(si->firi_base + FIRIRCR);
		cr &= ~FIRIRCR_RE;
		writel(cr, si->firi_base + FIRIRCR);

		writel(skb->len - 1, si->firi_base + FIRITCTR);

#ifdef FIRI_SDMA_TX
		/*
		 * Configure DMA Tx Channel for source and destination addresses,
		 * Number of bytes in SK buffer to transfer and Transfer complete
		 * callback function.
		 */
		si->dma_tx_buff_len = skb_len;
		si->dma_tx_buff_phy =
		    dma_map_single(si->dev, p, skb_len, DMA_TO_DEVICE);

		dma_request.num_of_bytes = skb_len;
		dma_request.dst_addr = si->firi_res->start + FIRITXFIFO;
		dma_request.src_addr = si->dma_tx_buff_phy;

		mxc_dma_config(si->txdma_ch, &dma_request, 1,
			       MXC_DMA_MODE_WRITE);

		mxc_dma_enable(si->txdma_ch);
#endif
		cr = readl(si->firi_base + FIRITCR);
		cr |= FIRITCR_TCIE;
		writel(cr, si->firi_base + FIRITCR);

		cr |= FIRITCR_TE;
		writel(cr, si->firi_base + FIRITCR);

#ifndef FIRI_SDMA_TX
		for (i = 0; i < skb->len;) {
			sr = readl(si->firi_base + FIRITSR);
			/* TFP = number of bytes in the TX FIFO for the
			 * Transmitter
			 * */
			if ((sr >> 8) < 128) {
				writeb(*p, si->firi_base + FIRITXFIFO);
				p++;
				i++;
			}
		}
#endif
	}

	dev->trans_start = jiffies;
	return 0;
}

/*!
 * This function handles network interface ioctls passed to this driver..
 *
 * @param   dev   net device structure
 * @param   ifreq user request data
 * @param   cmd   command issued
 *
 * @return  The function returns 0 on success and a non-zero value on
 *           failure.
 */
static int mxc_irda_ioctl(struct net_device *dev, struct ifreq *ifreq, int cmd)
{
	struct if_irda_req *rq = (struct if_irda_req *)ifreq;
	struct mxc_irda *si = netdev_priv(dev);
	int ret = -EOPNOTSUPP;

	switch (cmd) {
		/* This function will be used by IrLAP to change the speed */
	case SIOCSBANDWIDTH:
		dev_dbg(si->dev, "%s:with cmd SIOCSBANDWIDTH\n", __FUNCTION__);
		if (capable(CAP_NET_ADMIN)) {
			/*
			 * We are unable to set the speed if the
			 * device is not running.
			 */
			if (si->open) {
				ret = mxc_irda_set_speed(si, rq->ifr_baudrate);
			} else {
				dev_err(si->dev, "mxc_ir_ioctl: SIOCSBANDWIDTH:\
				 !netif_running\n");
				ret = 0;
			}
		}
		break;
	case SIOCSMEDIABUSY:
		dev_dbg(si->dev, "%s:with cmd SIOCSMEDIABUSY\n", __FUNCTION__);
		ret = -EPERM;
		if (capable(CAP_NET_ADMIN)) {
			irda_device_set_media_busy(dev, TRUE);
			ret = 0;
		}
		break;
	case SIOCGRECEIVING:
		rq->ifr_receiving =
		    IS_SIR(si) ? si->rx_buff.state != OUTSIDE_FRAME : 0;
		ret = 0;
		break;
	default:
		break;
	}
	return ret;
}

/*!
 * Kernel interface routine to get current statistics of the device
 * which includes the number bytes/packets transmitted/received,
 * receive errors, CRC errors, framing errors etc.
 *
 * @param  dev  the net_device structure
 *
 * @return This function returns IrDA network statistics
 */
static struct net_device_stats *mxc_irda_stats(struct net_device *dev)
{
	struct mxc_irda *si = netdev_priv(dev);
	return &si->stats;
}

/*!
 * FIRI init function
 *
 * @param si  FIRI device specific structure.
 */
void mxc_irda_firi_init(struct mxc_irda *si)
{
	unsigned int firi_baud, osf = 6;
	unsigned int tcr, rcr, cr;

	si->firi_clk = clk_get(si->dev, "firi_clk");
	firi_baud = clk_round_rate(si->firi_clk, 48004500);
	if ((firi_baud < 47995500) ||
	    (clk_set_rate(si->firi_clk, firi_baud) < 0)) {
		dev_err(si->dev, "Unable to set FIR clock to 48MHz.\n");
		return;
	}
	clk_enable(si->firi_clk);

	writel(0xFFFF, si->firi_base + FIRITSR);
	writel(0xFFFF, si->firi_base + FIRIRSR);
	writel(0x00, si->firi_base + FIRITCR);
	writel(0x00, si->firi_base + FIRIRCR);

	/* set _BL & _OSF */
	cr = (osf - 1) | (16 << 5);
	writel(cr, si->firi_base + FIRICR);

#ifdef FIRI_SDMA_TX
	tcr =
	    FIRITCR_TDT_FIR | FIRITCR_TM_FIR | FIRITCR_TCIE |
	    FIRITCR_PCF | FIRITCR_PC;
#else
	tcr = FIRITCR_TM_FIR | FIRITCR_TCIE | FIRITCR_PCF | FIRITCR_PC;
#endif

#ifdef FIRI_SDMA_RX
	rcr =
	    FIRIRCR_RPEDE | FIRIRCR_RM_FIR | FIRIRCR_RDT_FIR |
	    FIRIRCR_RPA | FIRIRCR_RPP;
#else
	rcr =
	    FIRIRCR_RPEDE | FIRIRCR_RM_FIR | FIRIRCR_RDT_FIR | FIRIRCR_RPEIE |
	    FIRIRCR_RPA | FIRIRCR_PAIE | FIRIRCR_RFOIE | FIRIRCR_RPP;
#endif

	writel(tcr, si->firi_base + FIRITCR);
	writel(rcr, si->firi_base + FIRIRCR);
	cr = 0;
	writel(cr, si->firi_base + FIRITCTR);
}

/*!
 * This function initialises the UART.
 *
 * @param   si  FIRI port specific structure.
 *
 * @return  The function returns 0 on success.
 */
static int mxc_irda_uart_init(struct mxc_irda *si)
{
	unsigned int per_clk;
	unsigned int num, denom, baud, ufcr = 0;
	unsigned int cr;
	int d = 1;
	int uart_ir_mux = 0;

	if (si->mxc_ir_plat)
		uart_ir_mux = si->mxc_ir_plat->uart_ir_mux;
	/*
	 * Clear Status Registers 1 and 2
	 **/
	writel(0xFFFF, si->uart_base + MXC_UARTUSR1);
	writel(0xFFFF, si->uart_base + MXC_UARTUSR2);

	/* Configure the IOMUX for the UART */
	gpio_firi_init();

	per_clk = clk_get_rate(si->uart_clk);
	baud = per_clk / 16;
	if (baud > 1500000) {
		baud = 1500000;
		d = per_clk / ((baud * 16) + 1000);
		if (d > 6) {
			d = 6;
		}
	}
	clk_enable(si->uart_clk);

	si->uart_clk_rate = per_clk / d;
	writel(si->uart_clk_rate / 1000, si->uart_base + MXC_UARTONEMS);

	writel(si->mxc_ir_plat->ir_rx_invert | MXC_UARTUCR4_IRSC,
	       si->uart_base + MXC_UARTUCR4);

	if (uart_ir_mux) {
		writel(MXC_UARTUCR3_RXDMUXSEL | si->mxc_ir_plat->ir_tx_invert |
		       MXC_UARTUCR3_DSR, si->uart_base + MXC_UARTUCR3);
	} else {
		writel(si->mxc_ir_plat->ir_tx_invert | MXC_UARTUCR3_DSR,
		       si->uart_base + MXC_UARTUCR3);
	}

	writel(MXC_UARTUCR2_IRTS | MXC_UARTUCR2_CTS | MXC_UARTUCR2_WS |
	       MXC_UARTUCR2_ATEN | MXC_UARTUCR2_TXEN | MXC_UARTUCR2_RXEN,
	       si->uart_base + MXC_UARTUCR2);
	/* Wait till we are out of software reset */
	do {
		cr = readl(si->uart_base + MXC_UARTUCR2);
	} while (!(cr & MXC_UARTUCR2_SRST));

	ufcr |= (UART4_UFCR_TXTL << MXC_UARTUFCR_TXTL_OFFSET) |
	    ((6 - d) << MXC_UARTUFCR_RFDIV_OFFSET) | UART4_UFCR_RXTL;
	writel(ufcr, si->uart_base + MXC_UARTUFCR);

	writel(MXC_UARTUCR1_UARTEN | MXC_UARTUCR1_IREN,
	       si->uart_base + MXC_UARTUCR1);

	baud = 9600;
	num = baud / 100 - 1;
	denom = si->uart_clk_rate / 1600 - 1;

	if ((denom < 65536) && (si->uart_clk_rate > 1600)) {
		writel(num, si->uart_base + MXC_UARTUBIR);
		writel(denom, si->uart_base + MXC_UARTUBMR);
	}

	writel(0x0000, si->uart_base + MXC_UARTUTS);
	return 0;

}

/*!
 * This function enables FIRI port.
 *
 * @param   si  FIRI port specific structure.
 *
 * @return  The function returns 0 on success and a non-zero value on
 *          failure.
 */
static int mxc_irda_startup(struct mxc_irda *si)
{
	int ret = 0;

	mxc_irda_uart_init(si);
	mxc_irda_firi_init(si);

	/* configure FIRI device for speed */
	ret = mxc_irda_set_speed(si, si->speed = 9600);

	return ret;
}

/*!
 * When an ifconfig is issued which changes the device flag to include
 * IFF_UP this function is called. It is only called when the change
 * occurs, not when the interface remains up. The function grabs the interrupt
 * resources and registers FIRI interrupt service routines, requests for DMA
 * channels, configures the DMA channel. It then initializes  the IOMUX
 * registers to configure the pins for FIRI signals and finally initializes the
 * various FIRI registers and enables the port for reception.
 *
 * @param   dev   net device structure that is being opened
 *
 * @return  The function returns 0 for a successful open and non-zero value
 *          on failure.
 */
static int mxc_irda_start(struct net_device *dev)
{
	struct mxc_irda *si = netdev_priv(dev);
	int err;
	int ints_muxed = 0;
	mxc_dma_device_t dev_id = 0;

	if (si->uart_irq == si->uart_irq1)
		ints_muxed = 1;

	si->speed = 9600;

	if (si->uart_irq == si->firi_irq) {
		err =
		    request_irq(si->uart_irq, mxc_irda_irq, 0, dev->name, dev);
		if (err) {
			dev_err(si->dev, "%s:Failed to request the IRQ\n",
				__FUNCTION__);
			return err;
		}
		/*
		 * The interrupt must remain disabled for now.
		 */
		disable_irq(si->uart_irq);
	} else {
		err =
		    request_irq(si->firi_irq, mxc_irda_irq, 0, dev->name, dev);
		if (err) {
			dev_err(si->dev, "%s:Failed to request FIRI IRQ\n",
				__FUNCTION__);
			return err;
		}
		/*
		 * The interrupt must remain disabled for now.
		 */
		disable_irq(si->firi_irq);
		if (ints_muxed) {

			err = request_irq(si->uart_irq, mxc_irda_irq, 0,
					  dev->name, dev);
			if (err) {
				dev_err(si->dev,
					"%s:Failed to request UART IRQ\n",
					__FUNCTION__);
				goto err_irq1;
			}
			/*
			 * The interrupt must remain disabled for now.
			 */
			disable_irq(si->uart_irq);
		} else {
			err = request_irq(si->uart_irq, mxc_irda_tx_irq, 0,
					  dev->name, dev);
			if (err) {
				dev_err(si->dev,
					"%s:Failed to request UART IRQ\n",
					__FUNCTION__);
				goto err_irq1;
			}
			err = request_irq(si->uart_irq1, mxc_irda_rx_irq, 0,
					  dev->name, dev);
			if (err) {
				dev_err(si->dev,
					"%s:Failed to request UART1 IRQ\n",
					__FUNCTION__);
				goto err_irq2;
			}
			/*
			 * The interrupts must remain disabled for now.
			 */
			disable_irq(si->uart_irq);
			disable_irq(si->uart_irq1);
		}
	}
#ifdef FIRI_SDMA_RX
	dev_id = MXC_DMA_FIR_RX;
	si->rxdma_ch = mxc_dma_request(dev_id, "MXC FIRI RX");
	if (si->rxdma_ch < 0) {
		dev_err(si->dev, "Cannot allocate FIR DMA channel\n");
		goto err_rx_dma;
	}
	mxc_dma_callback_set(si->rxdma_ch, mxc_irda_fir_dma_rx_irq,
			     (void *)dev_get_drvdata(si->dev));
#endif
#ifdef FIRI_SDMA_TX

	dev_id = MXC_DMA_FIR_TX;
	si->txdma_ch = mxc_dma_request(dev_id, "MXC FIRI TX");
	if (si->txdma_ch < 0) {
		dev_err(si->dev, "Cannot allocate FIR DMA channel\n");
		goto err_tx_dma;
	}
	mxc_dma_callback_set(si->txdma_ch, mxc_irda_fir_dma_tx_irq,
			     (void *)dev_get_drvdata(si->dev));
#endif
	/* Setup the serial port port for the initial speed. */
	err = mxc_irda_startup(si);
	if (err) {
		goto err_startup;
	}

	/* Open a new IrLAP layer instance. */
	si->irlap = irlap_open(dev, &si->qos, "mxc");
	err = -ENOMEM;
	if (!si->irlap) {
		goto err_irlap;
	}

	/* Now enable the interrupt and start the queue */
	si->open = 1;
	si->suspend = 0;

	if (si->uart_irq == si->firi_irq) {
		enable_irq(si->uart_irq);
	} else {
		enable_irq(si->firi_irq);
		if (ints_muxed == 1) {
			enable_irq(si->uart_irq);
		} else {
			enable_irq(si->uart_irq);
			enable_irq(si->uart_irq1);
		}
	}

	netif_start_queue(dev);
	return 0;

      err_irlap:
	si->open = 0;
	mxc_irda_disabledma(si);
      err_startup:
#ifdef FIRI_SDMA_TX
	mxc_dma_free(si->txdma_ch);
      err_tx_dma:
#endif
#ifdef FIRI_SDMA_RX
	mxc_dma_free(si->rxdma_ch);
      err_rx_dma:
#endif
	if (si->uart_irq1 && !ints_muxed)
		free_irq(si->uart_irq1, dev);
      err_irq2:
	if (si->uart_irq != si->firi_irq)
		free_irq(si->uart_irq, dev);
      err_irq1:
	if (si->firi_irq)
		free_irq(si->firi_irq, dev);
	return err;
}

/*!
 * This function is called when IFF_UP flag has been cleared by the user via
 * the ifconfig irda0 down command. This function stops any further
 * transmissions being queued, and then disables the interrupts.
 * Finally it resets the device.
 * @param   dev   the net_device structure
 *
 * @return  int   the function always returns 0 indicating a success.
 */
static int mxc_irda_stop(struct net_device *dev)
{
	struct mxc_irda *si = netdev_priv(dev);
	unsigned long flags;

	/* Stop IrLAP */
	if (si->irlap) {
		irlap_close(si->irlap);
		si->irlap = NULL;
	}

	netif_stop_queue(dev);

	/*Save flags and disable the FIRI interrupts.. */
	if (si->open) {
		local_irq_save(flags);
		disable_irq(si->uart_irq);
		free_irq(si->uart_irq, dev);
		if (si->uart_irq != si->firi_irq) {
			disable_irq(si->firi_irq);
			free_irq(si->firi_irq, dev);
			if (si->uart_irq1 != si->uart_irq) {
				disable_irq(si->uart_irq1);
				free_irq(si->uart_irq1, dev);
			}
		}
		local_irq_restore(flags);
		si->open = 0;
	}
#ifdef FIRI_SDMA_RX
	if (si->rxdma_ch) {
		mxc_dma_disable(si->rxdma_ch);
		mxc_dma_free(si->rxdma_ch);
		if (si->dma_rx_buff_phy) {
			dma_unmap_single(si->dev, si->dma_rx_buff_phy,
					 IRDA_FRAME_SIZE_LIMIT,
					 DMA_FROM_DEVICE);
			si->dma_rx_buff_phy = 0;
		}
		si->rxdma_ch = 0;
	}
	tasklet_kill(&dma_rx_tasklet);
#endif
#ifdef FIRI_SDMA_TX
	if (si->txdma_ch) {
		mxc_dma_disable(si->txdma_ch);
		mxc_dma_free(si->txdma_ch);
		if (si->dma_tx_buff_phy) {
			dma_unmap_single(si->dev, si->dma_tx_buff_phy,
					 si->dma_tx_buff_len, DMA_TO_DEVICE);
			si->dma_tx_buff_phy = 0;
		}
		si->txdma_ch = 0;
	}
#endif
	return 0;
}

#ifdef CONFIG_PM
/*!
 * This function is called to put the FIRI in a low power state. Refer to the
 * document driver-model/driver.txt in the kernel source tree for more
 * information.
 *
 * @param   pdev  the device structure used to give information on which FIRI
 *                to suspend
 * @param   state the power state the device is entering
 *
 * @return  The function always returns 0.
 */
static int mxc_irda_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct net_device *ndev = platform_get_drvdata(pdev);
	struct mxc_irda *si = netdev_priv(ndev);
	unsigned int cr;
	unsigned long flags;

	if (!si) {
		return 0;
	}
	if (si->suspend == 1) {
		dev_err(si->dev,
			" suspend - Device is already suspended ... \n");
		return 0;
	}
	if (si->open) {

		netif_device_detach(ndev);
		mxc_irda_disabledma(si);

		/*Save flags and disable the FIRI interrupts.. */
		local_irq_save(flags);
		disable_irq(si->uart_irq);
		if (si->uart_irq != si->firi_irq) {
			disable_irq(si->firi_irq);
			if (si->uart_irq != si->uart_irq1) {
				disable_irq(si->uart_irq1);
			}
		}
		local_irq_restore(flags);

		/* Disable Tx and Rx and then disable the UART clock */
		cr = readl(si->uart_base + MXC_UARTUCR2);
		cr &= ~(MXC_UARTUCR2_TXEN | MXC_UARTUCR2_RXEN);
		writel(cr, si->uart_base + MXC_UARTUCR2);
		cr = readl(si->uart_base + MXC_UARTUCR1);
		cr &= ~MXC_UARTUCR1_UARTEN;
		writel(cr, si->uart_base + MXC_UARTUCR1);
		clk_disable(si->uart_clk);

		/*Disable Tx and Rx for FIRI and then disable the FIRI clock.. */
		cr = readl(si->firi_base + FIRITCR);
		cr &= ~FIRITCR_TE;
		writel(cr, si->firi_base + FIRITCR);
		cr = readl(si->firi_base + FIRIRCR);
		cr &= ~FIRIRCR_RE;
		writel(cr, si->firi_base + FIRIRCR);
		clk_disable(si->firi_clk);

		gpio_firi_inactive();

		si->suspend = 1;
		si->open = 0;
	}
	return 0;
}

/*!
 * This function is called to bring the FIRI back from a low power state. Refer
 * to the document driver-model/driver.txt in the kernel source tree for more
 * information.
 *
 * @param   pdev  the device structure used to give information on which FIRI
 *                to resume
 *
 * @return  The function always returns 0.
 */
static int mxc_irda_resume(struct platform_device *pdev)
{
	struct net_device *ndev = platform_get_drvdata(pdev);
	struct mxc_irda *si = netdev_priv(ndev);
	unsigned long flags;

	if (!si) {
		return 0;
	}

	if (si->suspend == 1 && !si->open) {

		/*Initialise the UART first */
		clk_enable(si->uart_clk);

		/*Now init FIRI */
		gpio_firi_active(si->firi_base + FIRITCR, FIRITCR_TPP);
		mxc_irda_startup(si);

		/* Enable the UART and FIRI interrupts.. */
		local_irq_save(flags);
		enable_irq(si->uart_irq);
		if (si->uart_irq != si->firi_irq) {
			enable_irq(si->firi_irq);
			if (si->uart_irq != si->uart_irq1) {
				enable_irq(si->uart_irq1);
			}
		}
		local_irq_restore(flags);

		/* Let the kernel know that we are alive and kicking.. */
		netif_device_attach(ndev);

		si->suspend = 0;
		si->open = 1;
	}
	return 0;
}
#else
#define mxc_irda_suspend NULL
#define mxc_irda_resume  NULL
#endif

static int mxc_irda_init_iobuf(iobuff_t *io, int size)
{
	io->head = kmalloc(size, GFP_KERNEL | GFP_DMA);
	if (io->head != NULL) {
		io->truesize = size;
		io->in_frame = FALSE;
		io->state = OUTSIDE_FRAME;
		io->data = io->head;
	}
	return io->head ? 0 : -ENOMEM;

}

static struct net_device_ops mxc_irda_ops = {
	.ndo_start_xmit = mxc_irda_hard_xmit,
	.ndo_open = mxc_irda_start,
	.ndo_stop = mxc_irda_stop,
	.ndo_do_ioctl = mxc_irda_ioctl,
	.ndo_get_stats = mxc_irda_stats,
};

/*!
 * This function is called during the driver binding process.
 * This function requests for memory, initializes net_device structure and
 * registers with kernel.
 *
 * @param   pdev  the device structure used to store device specific
 *                information that is used by the suspend, resume and remove
 *                functions
 *
 * @return  The function returns 0 on success and a non-zero value on failure
 */
static int mxc_irda_probe(struct platform_device *pdev)
{
	struct net_device *dev;
	struct mxc_irda *si;
	struct resource *uart_res, *firi_res;
	int uart_irq, firi_irq, uart_irq1;
	unsigned int baudrate_mask = 0;
	int err;

	uart_res = &pdev->resource[0];
	uart_irq = pdev->resource[1].start;

	firi_res = &pdev->resource[2];
	firi_irq = pdev->resource[3].start;

	uart_irq1 = pdev->resource[4].start;

	if (!uart_res || uart_irq == NO_IRQ || !firi_res || firi_irq == NO_IRQ) {
		dev_err(&pdev->dev, "Unable to find resources\n");
		return -ENXIO;
	}

	err =
	    request_mem_region(uart_res->start, SZ_16K,
			       "MXC_IRDA") ? 0 : -EBUSY;
	if (err) {
		dev_err(&pdev->dev, "Failed to request UART memory region\n");
		return -ENOMEM;
	}

	err =
	    request_mem_region(firi_res->start, SZ_16K,
			       "MXC_IRDA") ? 0 : -EBUSY;
	if (err) {
		dev_err(&pdev->dev, "Failed to request FIRI  memory region\n");
		goto err_mem_1;
	}

	dev = alloc_irdadev(sizeof(struct mxc_irda));
	if (!dev) {
		goto err_mem_2;
	}

	si = netdev_priv(dev);
	si->dev = &pdev->dev;

	si->mxc_ir_plat = pdev->dev.platform_data;
	si->uart_clk = si->mxc_ir_plat->uart_clk;

	si->uart_res = uart_res;
	si->firi_res = firi_res;
	si->uart_irq = uart_irq;
	si->firi_irq = firi_irq;
	si->uart_irq1 = uart_irq1;

	si->uart_base = ioremap(uart_res->start, SZ_16K);
	si->firi_base = ioremap(firi_res->start, SZ_16K);

	if (!(si->uart_base || si->firi_base)) {
		err = -ENOMEM;
		goto err_mem_3;
	}

	/*
	 * Initialise the SIR buffers
	 */
	err = mxc_irda_init_iobuf(&si->rx_buff, UART_BUFF_SIZE);
	if (err) {
		goto err_mem_4;
	}

	err = mxc_irda_init_iobuf(&si->tx_buff, UART_BUFF_SIZE);
	if (err) {
		goto err_mem_5;
	}

	dev->netdev_ops = &mxc_irda_ops;

	irda_init_max_qos_capabilies(&si->qos);

	/*
	 * We support
	 * SIR(9600, 19200,38400, 57600 and 115200 bps)
	 * FIR(4 Mbps)
	 * Min Turn Time set to 1ms or greater.
	 */
	baudrate_mask |= IR_9600 | IR_19200 | IR_38400 | IR_57600 | IR_115200;
	baudrate_mask |= IR_4000000 << 8;

	si->qos.baud_rate.bits &= baudrate_mask;
	si->qos.min_turn_time.bits = 0x7;

	irda_qos_bits_to_value(&si->qos);

#ifdef FIRI_SDMA_RX
	si->tskb = NULL;
	tasklet_init(&dma_rx_tasklet, mxc_irda_rx_task, (unsigned long)si);
#endif
	err = register_netdev(dev);
	if (err == 0) {
		platform_set_drvdata(pdev, dev);
	} else {
		kfree(si->tx_buff.head);
	      err_mem_5:
		kfree(si->rx_buff.head);
	      err_mem_4:
		iounmap(si->uart_base);
		iounmap(si->firi_base);
	      err_mem_3:
		free_netdev(dev);
	      err_mem_2:
		release_mem_region(firi_res->start, SZ_16K);
	      err_mem_1:
		release_mem_region(uart_res->start, SZ_16K);
	}
	return err;
}

/*!
 * Dissociates the driver from the FIRI device. Removes the appropriate FIRI
 * port structure from the kernel.
 *
 * @param   pdev  the device structure used to give information on which FIRI
 *                to remove
 *
 * @return  The function always returns 0.
 */
static int mxc_irda_remove(struct platform_device *pdev)
{
	struct net_device *dev = platform_get_drvdata(pdev);
	struct mxc_irda *si = netdev_priv(dev);

	if (si->uart_base)
		iounmap(si->uart_base);
	if (si->firi_base)
		iounmap(si->firi_base);
	if (si->firi_res->start)
		release_mem_region(si->firi_res->start, SZ_16K);
	if (si->uart_res->start)
		release_mem_region(si->uart_res->start, SZ_16K);
	if (si->tx_buff.head)
		kfree(si->tx_buff.head);
	if (si->rx_buff.head)
		kfree(si->rx_buff.head);

	platform_set_drvdata(pdev, NULL);
	unregister_netdev(dev);
	free_netdev(dev);

	return 0;
}

/*!
 * This structure contains pointers to the power management callback functions.
 */
static struct platform_driver mxcir_driver = {
	.driver = {
		   .name = "mxcir",
		   },
	.probe = mxc_irda_probe,
	.remove = mxc_irda_remove,
	.suspend = mxc_irda_suspend,
	.resume = mxc_irda_resume,
};

/*!
 * This function is used to initialize the FIRI driver module. The function
 * registers the power management callback functions with the kernel and also
 * registers the FIRI callback functions.
 *
 * @return  The function returns 0 on success and a non-zero value on failure.
 */
static int __init mxc_irda_init(void)
{
	return platform_driver_register(&mxcir_driver);
}

/*!
 * This function is used to cleanup all resources before the driver exits.
 */
static void __exit mxc_irda_exit(void)
{
	platform_driver_unregister(&mxcir_driver);
}

module_init(mxc_irda_init);
module_exit(mxc_irda_exit);

MODULE_AUTHOR("Freescale Semiconductor");
MODULE_DESCRIPTION("MXC IrDA(SIR/FIR) driver");
MODULE_LICENSE("GPL");
