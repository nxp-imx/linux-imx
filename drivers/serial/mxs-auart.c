/*
 * Freescale STMP37XX/STMP378X Application UART driver
 *
 * Author: dmitry pervushin <dimka@embeddedalley.com>
 *
 * Copyright 2008-2011 Freescale Semiconductor, Inc.
 * Copyright 2008 Embedded Alley Solutions, Inc All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/console.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/wait.h>
#include <linux/tty.h>
#include <linux/tty_driver.h>
#include <linux/tty_flip.h>
#include <linux/serial.h>
#include <linux/serial_core.h>
#include <linux/platform_device.h>
#include <linux/device.h>
#include <linux/clk.h>
#include <linux/cpufreq.h>
#include <linux/dma-mapping.h>
#include <linux/delay.h>
#include <linux/uaccess.h>

#include <asm/cacheflush.h>

#include <mach/hardware.h>
#include <mach/device.h>
#include <mach/dmaengine.h>

#include "regs-uartapp.h"

#define MXS_AUART_MAJOR	242
#define MXS_AUART_RX_THRESHOLD 16

static struct uart_driver auart_driver;

struct mxs_auart_port {
	struct uart_port port;

	unsigned int flags;
#define MXS_AUART_PORT_OPEN	0x80000000
#define MXS_AUART_PORT_DMA_MODE	0x80000000
	unsigned int ctrl;

	unsigned int irq[3];

	struct clk *clk;
	struct device *dev;
	unsigned int dma_rx_chan;
	unsigned int dma_tx_chan;
	unsigned int dma_rx_buffer_size;
	struct list_head rx_done;
	struct list_head free;
	struct mxs_dma_desc *tx;
	struct tasklet_struct rx_task;
};

static void mxs_auart_stop_tx(struct uart_port *u);
static void mxs_auart_submit_tx(struct mxs_auart_port *s, int size);
static void mxs_auart_submit_rx(struct mxs_auart_port *s);

static inline struct mxs_auart_port *to_auart_port(struct uart_port *u)
{
	return container_of(u, struct mxs_auart_port, port);
}

static inline void mxs_auart_tx_chars(struct mxs_auart_port *s)
{
	struct circ_buf *xmit = &s->port.state->xmit;

	if (s->flags & MXS_AUART_PORT_DMA_MODE) {
		int i = 0, size;
		char *buffer = s->tx->buffer;

		if (mxs_dma_desc_pending(s->tx))
			return;
		while (!uart_circ_empty(xmit) && !uart_tx_stopped(&s->port)) {
			if (i >= PAGE_SIZE)
				break;
			if (s->port.x_char) {
				buffer[i++] = s->port.x_char;
				s->port.x_char = 0;
				continue;
			}
			size = min_t(u32, PAGE_SIZE - i,
				     CIRC_CNT_TO_END(xmit->head,
						     xmit->tail,
						     UART_XMIT_SIZE));
			memcpy(buffer + i, xmit->buf + xmit->tail, size);
			xmit->tail = (xmit->tail + size) & (UART_XMIT_SIZE - 1);
			if (uart_circ_chars_pending(xmit) < WAKEUP_CHARS)
				uart_write_wakeup(&s->port);
			i += size;
		}
		if (i)
			mxs_auart_submit_tx(s, i);
		else {
			if (uart_tx_stopped(&s->port))
				mxs_auart_stop_tx(&s->port);
		}
		return;
	}

	while (!(__raw_readl(s->port.membase + HW_UARTAPP_STAT) &
		 BM_UARTAPP_STAT_TXFF)) {
		if (s->port.x_char) {
			__raw_writel(s->port.x_char,
				     s->port.membase + HW_UARTAPP_DATA);
			s->port.x_char = 0;
			continue;
		}
		if (!uart_circ_empty(xmit) && !uart_tx_stopped(&s->port)) {
			__raw_writel(xmit->buf[xmit->tail],
				     s->port.membase + HW_UARTAPP_DATA);
			xmit->tail = (xmit->tail + 1) & (UART_XMIT_SIZE - 1);
			if (uart_circ_chars_pending(xmit) < WAKEUP_CHARS)
				uart_write_wakeup(&s->port);
		} else
			break;
	}
	if (uart_circ_empty(&(s->port.state->xmit)))
		__raw_writel(BM_UARTAPP_INTR_TXIEN,
			     s->port.membase + HW_UARTAPP_INTR_CLR);
	else
		__raw_writel(BM_UARTAPP_INTR_TXIEN,
			     s->port.membase + HW_UARTAPP_INTR_SET);

	if (uart_tx_stopped(&s->port))
		mxs_auart_stop_tx(&s->port);
}

static inline unsigned int
mxs_auart_rx_char(struct mxs_auart_port *s, unsigned int stat, u8 c)
{
	int flag;

	flag = TTY_NORMAL;
	if (stat & BM_UARTAPP_STAT_BERR) {
		stat &= ~BM_UARTAPP_STAT_BERR;
		s->port.icount.brk++;
		if (uart_handle_break(&s->port))
			return stat;
		flag = TTY_BREAK;
	} else if (stat & BM_UARTAPP_STAT_PERR) {
		stat &= ~BM_UARTAPP_STAT_PERR;
		s->port.icount.parity++;
		flag = TTY_PARITY;
	} else if (stat & BM_UARTAPP_STAT_FERR) {
		stat &= ~BM_UARTAPP_STAT_FERR;
		s->port.icount.frame++;
		flag = TTY_FRAME;
	}

	if (stat & BM_UARTAPP_STAT_OERR)
		s->port.icount.overrun++;

	if (uart_handle_sysrq_char(&s->port, c))
		return stat;

	uart_insert_char(&s->port, stat, BM_UARTAPP_STAT_OERR, c, flag);
	return stat;
}


static void dma_rx_do_tasklet(unsigned long arg)
{
	struct mxs_auart_port *s = (struct mxs_auart_port *) arg;
	struct tty_struct *tty = s->port.state->port.tty;
	u32 stat = 0;
	int count;
	struct list_head *p, *q;
	int flag;
	LIST_HEAD(list);
	struct mxs_dma_desc *pdesc;

	mxs_dma_cooked(s->dma_rx_chan, &list);
	stat = __raw_readl(s->port.membase + HW_UARTAPP_STAT);

	flag = TTY_NORMAL;
	if (stat & BM_UARTAPP_STAT_BERR) {
		stat &= ~BM_UARTAPP_STAT_BERR;
		s->port.icount.brk++;
		if (uart_handle_break(&s->port))
			return;
		flag = TTY_BREAK;
	} else if (stat & BM_UARTAPP_STAT_PERR) {
		stat &= ~BM_UARTAPP_STAT_PERR;
		s->port.icount.parity++;
		flag = TTY_PARITY;
	} else if (stat & BM_UARTAPP_STAT_FERR) {
		stat &= ~BM_UARTAPP_STAT_FERR;
		s->port.icount.frame++;
		flag = TTY_FRAME;
	}

	if (stat & BM_UARTAPP_STAT_OERR)
		s->port.icount.overrun++;

	list_for_each_safe(p, q, &list) {
		list_del(p);
		pdesc = list_entry(p, struct mxs_dma_desc, node);
		count = stat & BM_UARTAPP_STAT_RXCOUNT;
		tty_insert_flip_string(tty, pdesc->buffer, count);
		if (flag != TTY_NORMAL)
			tty_insert_flip_char(tty, 0, flag);
		list_add(p, &s->free);
	}
	mxs_auart_submit_rx(s);
	__raw_writel(stat, s->port.membase + HW_UARTAPP_STAT);
	tty_flip_buffer_push(tty);
}

static void mxs_auart_rx_chars(struct mxs_auart_port *s)
{
	u8 c;
	struct tty_struct *tty = s->port.state->port.tty;
	u32 stat = 0;

	if (s->flags & MXS_AUART_PORT_DMA_MODE) {
		tasklet_schedule(&s->rx_task);
		return;
	}
	for (;;) {
		stat = __raw_readl(s->port.membase + HW_UARTAPP_STAT);
		if (stat & BM_UARTAPP_STAT_RXFE)
			break;
		c = __raw_readl(s->port.membase + HW_UARTAPP_DATA);
		stat = mxs_auart_rx_char(s, stat, c);
		__raw_writel(stat, s->port.membase + HW_UARTAPP_STAT);
	}
	__raw_writel(stat, s->port.membase + HW_UARTAPP_STAT);
	tty_flip_buffer_push(tty);
}

/* Allocate and initialize rx and tx DMA chains */
static int mxs_auart_dma_init(struct mxs_auart_port *s)
{
	int ret, i;
	struct list_head *p, *n;
	struct mxs_dma_desc *pdesc;

	ret = mxs_dma_request(s->dma_rx_chan, s->dev, dev_name(s->dev));
	if (ret)
		goto fail_get_dma_rx;
	ret = mxs_dma_request(s->dma_tx_chan, s->dev, dev_name(s->dev));
	if (ret)
		goto fail_get_dma_tx;
	ret = -ENOMEM;
	INIT_LIST_HEAD(&s->rx_done);
	INIT_LIST_HEAD(&s->free);
	s->tx = NULL;


	for (i = 0; i < 5; i++) {
		pdesc = mxs_dma_alloc_desc();
		if (pdesc == NULL || IS_ERR(pdesc))
			goto fail_alloc_desc;

		if (s->tx == NULL) {
			pdesc->buffer = dma_alloc_coherent(s->dev, PAGE_SIZE,
						   &pdesc->cmd.address,
						   GFP_DMA);
			if (pdesc->buffer == NULL)
				goto fail_alloc_desc;
			s->tx = pdesc;
		} else {
			pdesc->buffer = dma_alloc_coherent(s->dev,
						s->dma_rx_buffer_size,
						&pdesc->cmd.address,
						GFP_DMA);
			if (pdesc->buffer == NULL)
				goto fail_alloc_desc;
			list_add_tail(&pdesc->node, &s->free);
		}
	}
	/*
	   Tell DMA to select UART.
	   Both DMA channels are shared between app UART and IrDA.
	   Target id of 0 means UART, 1 means IrDA
	 */
	mxs_dma_set_target(s->dma_rx_chan, 0);
	mxs_dma_set_target(s->dma_tx_chan, 0);

	mxs_dma_enable_irq(s->dma_rx_chan, 1);
	mxs_dma_enable_irq(s->dma_tx_chan, 1);

	/* Initialize RX tasklet */
	tasklet_init(&s->rx_task, dma_rx_do_tasklet, (unsigned long)s);

	return 0;
fail_alloc_desc:
	if (s->tx) {
		if (s->tx->buffer)
			dma_free_coherent(s->dev,
					  PAGE_SIZE,
					  s->tx->buffer,
					  s->tx->cmd.address);
		s->tx->buffer = NULL;
		mxs_dma_free_desc(s->tx);
		s->tx = NULL;
	}
	list_for_each_safe(p, n, &s->free) {
		list_del(p);
		pdesc = list_entry(p, struct mxs_dma_desc, node);
		if (pdesc->buffer)
			dma_free_coherent(s->dev,
					  s->dma_rx_buffer_size,
					  pdesc->buffer,
					  pdesc->cmd.address);
		pdesc->buffer = NULL;
		mxs_dma_free_desc(pdesc);
	}
	mxs_dma_release(s->dma_tx_chan, s->dev);
fail_get_dma_tx:
	mxs_dma_release(s->dma_rx_chan, s->dev);
fail_get_dma_rx:
	WARN_ON(ret);
	return ret;
}

static void mxs_auart_dma_exit(struct mxs_auart_port *s)
{
	struct list_head *p, *n;
		LIST_HEAD(list);
	struct mxs_dma_desc *pdesc;

	mxs_dma_enable_irq(s->dma_rx_chan, 0);
	mxs_dma_enable_irq(s->dma_tx_chan, 0);

	mxs_dma_disable(s->dma_tx_chan);
	mxs_dma_disable(s->dma_rx_chan);

	mxs_dma_get_cooked(s->dma_tx_chan, &list);
	mxs_dma_get_cooked(s->dma_rx_chan, &s->free);

	mxs_dma_release(s->dma_tx_chan, s->dev);
	mxs_dma_release(s->dma_rx_chan, s->dev);

	if (s->tx) {
		if (s->tx->buffer)
			dma_free_coherent(s->dev,
					  PAGE_SIZE,
					  s->tx->buffer,
					  s->tx->cmd.address);
		s->tx->buffer = NULL;
		mxs_dma_free_desc(s->tx);
		s->tx = NULL;
	}
	list_for_each_safe(p, n, &s->free) {
		list_del(p);
		pdesc = list_entry(p, struct mxs_dma_desc, node);
		if (pdesc->buffer)
			dma_free_coherent(s->dev,
					  s->dma_rx_buffer_size,
					  pdesc->buffer,
					  pdesc->cmd.address);
		pdesc->buffer = NULL;
		mxs_dma_free_desc(pdesc);
	}
}

static void mxs_auart_submit_rx(struct mxs_auart_port *s)
{
	int ret;
	unsigned int pio_value;
	struct list_head *p, *n;
	struct mxs_dma_desc *pdesc;

	pio_value = BM_UARTAPP_CTRL0_RXTO_ENABLE |
		     BF_UARTAPP_CTRL0_RXTIMEOUT(0x80) |
		     BF_UARTAPP_CTRL0_XFER_COUNT(s->dma_rx_buffer_size);

	list_for_each_safe(p, n, &s->free) {
		list_del(p);
		pdesc = list_entry(p, struct mxs_dma_desc, node);
		pdesc->cmd.cmd.bits.bytes = s->dma_rx_buffer_size;
		pdesc->cmd.cmd.bits.terminate_flush = 1;
		pdesc->cmd.cmd.bits.pio_words = 1;
		pdesc->cmd.cmd.bits.wait4end = 1;
		pdesc->cmd.cmd.bits.dec_sem = 1;
		pdesc->cmd.cmd.bits.irq = 1;
		pdesc->cmd.cmd.bits.chain = 1;
		pdesc->cmd.cmd.bits.command = DMA_WRITE;
		pdesc->cmd.pio_words[0] = pio_value;
		ret = mxs_dma_desc_append(s->dma_rx_chan, pdesc);
		if (ret)
			pr_info("%s append dma desc, %d\n", __func__, ret);
	}
	ret = mxs_dma_enable(s->dma_rx_chan);
	if (ret)
		pr_info("%s enable dma desc, %d\n", __func__, ret);
}

static irqreturn_t mxs_auart_irq_dma_rx(int irq, void *context)
{
	struct mxs_auart_port *s = context;

	mxs_dma_ack_irq(s->dma_rx_chan);
	mxs_auart_rx_chars(s);
	return IRQ_HANDLED;
}

static void mxs_auart_submit_tx(struct mxs_auart_port *s, int size)
{
	int ret;
	struct mxs_dma_desc *d = s->tx;

	d->cmd.pio_words[0] = BF_UARTAPP_CTRL1_XFER_COUNT(size);
	d->cmd.cmd.bits.bytes = size;
	d->cmd.cmd.bits.pio_words = 1;
	d->cmd.cmd.bits.wait4end = 1;
	d->cmd.cmd.bits.dec_sem = 1;
	d->cmd.cmd.bits.irq = 1;
	d->cmd.cmd.bits.command = DMA_READ;
	ret = mxs_dma_desc_append(s->dma_tx_chan, s->tx);
	if (ret)
		pr_info("append dma desc, %d\n", ret);

	ret = mxs_dma_enable(s->dma_tx_chan);
	if (ret)
		pr_info("enable dma desc, %d\n", ret);
}

static irqreturn_t mxs_auart_irq_dma_tx(int irq, void *context)
{
	struct mxs_auart_port *s = context;

	LIST_HEAD(list);
	mxs_dma_ack_irq(s->dma_tx_chan);
	mxs_dma_cooked(s->dma_tx_chan, &list);
	mxs_auart_tx_chars(s);
	return IRQ_HANDLED;
}

static int mxs_auart_request_port(struct uart_port *u)
{
	struct mxs_auart_port *s = to_auart_port(u);

	if (!request_mem_region((u32)u->mapbase, SZ_4K, dev_name(s->dev)))
		return -EBUSY;
	return 0;

}

static int mxs_auart_verify_port(struct uart_port *u,
				    struct serial_struct *ser)
{
	if (u->type != PORT_UNKNOWN && u->type != PORT_IMX)
		return -EINVAL;
	return 0;
}

static void mxs_auart_config_port(struct uart_port *u, int flags)
{
}

static const char *mxs_auart_type(struct uart_port *u)
{
	struct mxs_auart_port *s = to_auart_port(u);

	return dev_name(s->dev);
}

static void mxs_auart_release_port(struct uart_port *u)
{
	release_mem_region(u->mapbase, SZ_4K);
}

static void mxs_auart_set_mctrl(struct uart_port *u, unsigned mctrl)
{
	struct mxs_auart_port *s = to_auart_port(u);

	u32 ctrl = __raw_readl(u->membase + HW_UARTAPP_CTRL2);

	ctrl &= ~BM_UARTAPP_CTRL2_RTS;
	if (mctrl & TIOCM_RTS)
		ctrl |= BM_UARTAPP_CTRL2_RTS;
	s->ctrl = mctrl;
	__raw_writel(ctrl, u->membase + HW_UARTAPP_CTRL2);
}

static u32 mxs_auart_get_mctrl(struct uart_port *u)
{
	struct mxs_auart_port *s = to_auart_port(u);
	u32 stat = __raw_readl(u->membase + HW_UARTAPP_STAT);
	int ctrl2 = __raw_readl(u->membase + HW_UARTAPP_CTRL2);
	u32 mctrl = s->ctrl;

	mctrl &= ~TIOCM_CTS;
	if (stat & BM_UARTAPP_STAT_CTS)
		mctrl |= TIOCM_CTS;

	if (ctrl2 & BM_UARTAPP_CTRL2_RTS)
		mctrl |= TIOCM_RTS;

	return mctrl;
}

static void mxs_auart_settermios(struct uart_port *u,
				 struct ktermios *termios,
				 struct ktermios *old)
{
	u32 bm, ctrl, ctrl2, div;
	unsigned int cflag, baud;

	if (termios == NULL) {
		printk(KERN_ERR "Empty ktermios setting:!\n");
		return;
	}

	cflag = termios->c_cflag;

	ctrl = BM_UARTAPP_LINECTRL_FEN;
	ctrl2 = __raw_readl(u->membase + HW_UARTAPP_CTRL2);

	/* byte size */
	switch (cflag & CSIZE) {
	case CS5:
		bm = 0;
		break;
	case CS6:
		bm = 1;
		break;
	case CS7:
		bm = 2;
		break;
	case CS8:
		bm = 3;
		break;
	default:
		return;
	}

	ctrl |= BF_UARTAPP_LINECTRL_WLEN(bm);

	/* parity */
	if (cflag & PARENB) {
		ctrl |= BM_UARTAPP_LINECTRL_PEN;
		if ((cflag & PARODD) == 0)
			ctrl |= BM_UARTAPP_LINECTRL_EPS;
	}

	/* figure out the stop bits requested */
	if (cflag & CSTOPB)
		ctrl |= BM_UARTAPP_LINECTRL_STP2;

	/* figure out the hardware flow control settings */
	if (cflag & CRTSCTS)
		ctrl2 |= BM_UARTAPP_CTRL2_CTSEN | BM_UARTAPP_CTRL2_RTSEN;
	else
		ctrl2 &= ~BM_UARTAPP_CTRL2_CTSEN;

	/* set baud rate */
	baud = uart_get_baud_rate(u, termios, old, 0, u->uartclk);
	div = u->uartclk * 32 / baud;
	ctrl |= BF_UARTAPP_LINECTRL_BAUD_DIVFRAC(div & 0x3F);
	ctrl |= BF_UARTAPP_LINECTRL_BAUD_DIVINT(div >> 6);

	if ((cflag & CREAD) != 0)
		ctrl2 |= BM_UARTAPP_CTRL2_RXE;

	__raw_writel(ctrl, u->membase + HW_UARTAPP_LINECTRL);
	__raw_writel(ctrl2, u->membase + HW_UARTAPP_CTRL2);
}

static irqreturn_t mxs_auart_irq_handle(int irq, void *context)
{
	u32 istatus, istat;
	struct mxs_auart_port *s = context;
	u32 stat = __raw_readl(s->port.membase + HW_UARTAPP_STAT);

	istatus = istat = __raw_readl(s->port.membase + HW_UARTAPP_INTR);

	if (istat & BM_UARTAPP_INTR_CTSMIS) {
		uart_handle_cts_change(&s->port, stat & BM_UARTAPP_STAT_CTS);
		__raw_writel(BM_UARTAPP_INTR_CTSMIS,
				s->port.membase + HW_UARTAPP_INTR_CLR);
		istat &= ~BM_UARTAPP_INTR_CTSMIS;
	}
	if (istat & (BM_UARTAPP_INTR_RTIS | BM_UARTAPP_INTR_RXIS)) {
		mxs_auart_rx_chars(s);
		istat &= ~(BM_UARTAPP_INTR_RTIS | BM_UARTAPP_INTR_RXIS);
	}

	if (istat & BM_UARTAPP_INTR_TXIS) {
		mxs_auart_tx_chars(s);
		istat &= ~BM_UARTAPP_INTR_TXIS;
	}
	/* modem status interrupt bits are undefined
	after reset,and the hardware do not support
	DSRMIS,DCDMIS and RIMIS bit,so we should ingore
	them when they are pending. */
	if (istat & (BM_UARTAPP_INTR_ABDIS
		| BM_UARTAPP_INTR_OEIS
		| BM_UARTAPP_INTR_BEIS
		| BM_UARTAPP_INTR_PEIS
		| BM_UARTAPP_INTR_FEIS
		| BM_UARTAPP_INTR_RTIS
		| BM_UARTAPP_INTR_TXIS
		| BM_UARTAPP_INTR_RXIS
		| BM_UARTAPP_INTR_CTSMIS)) {
		dev_info(s->dev, "Unhandled status %x\n", istat);
	}
	__raw_writel(istatus & (BM_UARTAPP_INTR_ABDIS
		| BM_UARTAPP_INTR_OEIS
		| BM_UARTAPP_INTR_BEIS
		| BM_UARTAPP_INTR_PEIS
		| BM_UARTAPP_INTR_FEIS
		| BM_UARTAPP_INTR_RTIS
		| BM_UARTAPP_INTR_TXIS
		| BM_UARTAPP_INTR_RXIS
		| BM_UARTAPP_INTR_DSRMIS
		| BM_UARTAPP_INTR_DCDMIS
		| BM_UARTAPP_INTR_CTSMIS
		| BM_UARTAPP_INTR_RIMIS),
			s->port.membase + HW_UARTAPP_INTR_CLR);

	return IRQ_HANDLED;
}

static int mxs_auart_free_irqs(struct mxs_auart_port *s)
{
	int irqn = 0;

	for (irqn = 0; irqn < ARRAY_SIZE(s->irq); irqn++)
		free_irq(s->irq[irqn], s);
	return 0;
}

static int mxs_auart_request_irqs(struct mxs_auart_port *s)
{
	int err = 0;

	/*
	 * order counts. resources should be listed in the same order
	 */
	irq_handler_t handlers[] = {
		mxs_auart_irq_handle,
		mxs_auart_irq_dma_rx,
		mxs_auart_irq_dma_tx,
	};
	char *handlers_names[] = {
		"auart internal",
		"auart dma rx",
		"auart dma tx",
	};
	int irqn;

	for (irqn = 0; irqn < ARRAY_SIZE(handlers); irqn++) {
		err = request_irq(s->irq[irqn], handlers[irqn],
				  0, handlers_names[irqn], s);
		if (err)
			goto out;
	}
	return 0;
out:
	mxs_auart_free_irqs(s);
	return err;
}

static inline void mxs_auart_reset(struct uart_port *u)
{
	int i;
	unsigned int reg;

	__raw_writel(BM_UARTAPP_CTRL0_SFTRST,
		     u->membase + HW_UARTAPP_CTRL0_CLR);

	for (i = 0; i < 10000; i++) {
		reg = __raw_readl(u->membase + HW_UARTAPP_CTRL0);
		if (!(reg & BM_UARTAPP_CTRL0_SFTRST))
			break;
		udelay(3);
	}

	__raw_writel(BM_UARTAPP_CTRL0_CLKGATE,
		     u->membase + HW_UARTAPP_CTRL0_CLR);
}

static int mxs_auart_startup(struct uart_port *u)
{
	struct mxs_auart_port *s = to_auart_port(u);

	mxs_auart_reset(u);

	__raw_writel(BM_UARTAPP_CTRL2_UARTEN,
		     s->port.membase + HW_UARTAPP_CTRL2_SET);

	/* Enable the Application UART DMA bits. */
	if (s->flags & MXS_AUART_PORT_DMA_MODE) {
		int ret;
		ret = mxs_auart_dma_init(s);
		if (ret) {
			__raw_writel(BM_UARTAPP_CTRL2_UARTEN,
				     s->port.membase + HW_UARTAPP_CTRL2_CLR);
			return ret;
		}
		__raw_writel(BM_UARTAPP_CTRL2_TXDMAE | BM_UARTAPP_CTRL2_RXDMAE
			      | BM_UARTAPP_CTRL2_DMAONERR,
			     s->port.membase + HW_UARTAPP_CTRL2_SET);
		/* clear any pending interrupts */
		__raw_writel(0, s->port.membase + HW_UARTAPP_INTR);

		/* reset all dma channels */
		mxs_dma_reset(s->dma_tx_chan);
		mxs_dma_reset(s->dma_rx_chan);
	} else
		__raw_writel(BM_UARTAPP_INTR_RXIEN | BM_UARTAPP_INTR_RTIEN,
			     s->port.membase + HW_UARTAPP_INTR);

	__raw_writel(BM_UARTAPP_INTR_CTSMIEN,
		     s->port.membase + HW_UARTAPP_INTR_SET);

	/*
	 * Enable fifo so all four bytes of a DMA word are written to
	 * output (otherwise, only the LSB is written, ie. 1 in 4 bytes)
	 */
	__raw_writel(BM_UARTAPP_LINECTRL_FEN,
		     s->port.membase + HW_UARTAPP_LINECTRL_SET);

	if (s->flags & MXS_AUART_PORT_DMA_MODE)
		mxs_auart_submit_rx(s);
	return mxs_auart_request_irqs(s);
}

static void mxs_auart_shutdown(struct uart_port *u)
{
	struct mxs_auart_port *s = to_auart_port(u);

	__raw_writel(BM_UARTAPP_CTRL0_SFTRST,
		     s->port.membase + HW_UARTAPP_CTRL0_SET);

	if (s->flags & MXS_AUART_PORT_DMA_MODE)
		mxs_auart_dma_exit(s);
	else
		__raw_writel(BM_UARTAPP_INTR_RXIEN | BM_UARTAPP_INTR_RTIEN |
				BM_UARTAPP_INTR_CTSMIEN,
			     s->port.membase + HW_UARTAPP_INTR_CLR);
	mxs_auart_free_irqs(s);
}

static unsigned int mxs_auart_tx_empty(struct uart_port *u)
{
	struct mxs_auart_port *s = to_auart_port(u);

	if (s->flags & MXS_AUART_PORT_DMA_MODE)
		return mxs_dma_desc_pending(s->tx) ? 0 : TIOCSER_TEMT;

	if (__raw_readl(u->membase + HW_UARTAPP_STAT) &
	    BM_UARTAPP_STAT_TXFE)
		return TIOCSER_TEMT;
	else
		return 0;
}

static void mxs_auart_start_tx(struct uart_port *u)
{
	struct mxs_auart_port *s = to_auart_port(u);

	/* enable transmitter */
	__raw_writel(BM_UARTAPP_CTRL2_TXE, u->membase + HW_UARTAPP_CTRL2_SET);

	mxs_auart_tx_chars(s);
}

static void mxs_auart_stop_tx(struct uart_port *u)
{
	__raw_writel(BM_UARTAPP_CTRL2_TXE, u->membase + HW_UARTAPP_CTRL2_CLR);
}

static void mxs_auart_stop_rx(struct uart_port *u)
{
	__raw_writel(BM_UARTAPP_CTRL2_RXE, u->membase + HW_UARTAPP_CTRL2_CLR);
}

static void mxs_auart_break_ctl(struct uart_port *u, int ctl)
{
	if (ctl)
		__raw_writel(BM_UARTAPP_LINECTRL_BRK,
			     u->membase + HW_UARTAPP_LINECTRL_SET);
	else
		__raw_writel(BM_UARTAPP_LINECTRL_BRK,
			     u->membase + HW_UARTAPP_LINECTRL_CLR);
}

static void mxs_auart_enable_ms(struct uart_port *port)
{
	/* just empty */
}

static struct uart_ops mxs_auart_ops = {
	.tx_empty       = mxs_auart_tx_empty,
	.start_tx       = mxs_auart_start_tx,
	.stop_tx	= mxs_auart_stop_tx,
	.stop_rx	= mxs_auart_stop_rx,
	.enable_ms      = mxs_auart_enable_ms,
	.break_ctl      = mxs_auart_break_ctl,
	.set_mctrl	= mxs_auart_set_mctrl,
	.get_mctrl      = mxs_auart_get_mctrl,
	.startup	= mxs_auart_startup,
	.shutdown       = mxs_auart_shutdown,
	.set_termios    = mxs_auart_settermios,
	.type	   	= mxs_auart_type,
	.release_port   = mxs_auart_release_port,
	.request_port   = mxs_auart_request_port,
	.config_port    = mxs_auart_config_port,
	.verify_port    = mxs_auart_verify_port,
};
#ifdef CONFIG_SERIAL_MXS_AUART_CONSOLE
static struct mxs_auart_port auart_port[CONFIG_MXS_AUART_PORTS] = {};

static void
auart_console_write(struct console *co, const char *s, unsigned int count)
{
	struct uart_port *port;
	unsigned int status, old_cr;
	int i;

	if (co->index >	CONFIG_MXS_AUART_PORTS || co->index < 0)
		return;

	port = &auart_port[co->index].port;

	/* First save the CR then disable the interrupts */
	old_cr = __raw_readl(port->membase + HW_UARTAPP_CTRL2);
	__raw_writel(BM_UARTAPP_CTRL2_UARTEN | BM_UARTAPP_CTRL2_TXE,
		     port->membase + HW_UARTAPP_CTRL2_SET);

	/* Now, do each character */
	for (i = 0; i < count; i++) {
		do {
			status = __raw_readl(port->membase + HW_UARTAPP_STAT);
		} while (status & BM_UARTAPP_STAT_TXFF);

		__raw_writel(s[i], port->membase + HW_UARTAPP_DATA);
		if (s[i] == '\n') {
			do {
				status = __raw_readl(port->membase +
						     HW_UARTAPP_STAT);
			} while (status & BM_UARTAPP_STAT_TXFF);
			__raw_writel('\r', port->membase + HW_UARTAPP_DATA);
		}
	}

	/*
	 *      Finally, wait for transmitter to become empty
	 *      and restore the TCR
	 */
	do {
		status = __raw_readl(port->membase + HW_UARTAPP_STAT);
	} while (status & BM_UARTAPP_STAT_BUSY);
	__raw_writel(old_cr, port->membase + HW_UARTAPP_CTRL2);
}

static void __init
auart_console_get_options(struct uart_port *port, int *baud,
			  int *parity, int *bits)
{
	if (__raw_readl(port->membase + HW_UARTAPP_CTRL2)
				& BM_UARTAPP_CTRL2_UARTEN) {
		unsigned int lcr_h, quot;
		lcr_h = __raw_readl(port->membase + HW_UARTAPP_LINECTRL);

		*parity = 'n';
		if (lcr_h & BM_UARTAPP_LINECTRL_PEN) {
			if (lcr_h & BM_UARTAPP_LINECTRL_EPS)
				*parity = 'e';
			else
				*parity = 'o';
		}

		if ((lcr_h & BM_UARTAPP_LINECTRL_WLEN)
				== BF_UARTAPP_LINECTRL_WLEN(2))
			*bits = 7;
		else
			*bits = 8;

		quot = (((__raw_readl(port->membase + HW_UARTAPP_LINECTRL)
				& BM_UARTAPP_LINECTRL_BAUD_DIVINT))
				    >> (BP_UARTAPP_LINECTRL_BAUD_DIVINT - 6))
			| (((__raw_readl(port->membase + HW_UARTAPP_LINECTRL)
				& BM_UARTAPP_LINECTRL_BAUD_DIVFRAC))
					>> BP_UARTAPP_LINECTRL_BAUD_DIVFRAC);
		if (quot == 0)
			quot = 1;
		*baud = (port->uartclk << 2) / quot;
	}
}

static int __init auart_console_setup(struct console *co, char *options)
{
	struct mxs_auart_port *port;
	int baud = 115200;
	int bits = 8;
	int parity = 'n';
	int flow = 'n';
	/*
	 * Check whether an invalid uart number has been specified, and
	 * if so, search for the first available port that does have
	 * console support.
	 */
	if (co->index > CONFIG_MXS_AUART_PORTS || co->index < 0)
		return -EINVAL;

	port = &auart_port[co->index].port;

	if (port->port.membase == 0) {
		if (cpu_is_mx23()) {
			if (co->index == 1) {
				port->port.membase = IO_ADDRESS(0x8006C000);
				port->port.mapbase = 0x8006C000;
			} else {
				port->port.membase = IO_ADDRESS(0x8006E000);
				port->port.mapbase = 0x8006E000;
			}
		}

		port->port.fifosize = 16;
		port->port.ops = &mxs_auart_ops;
		port->port.flags = ASYNC_BOOT_AUTOCONF;
		port->port.line = 0;
	}
	mxs_auart_reset(port);

	__raw_writel(BM_UARTAPP_CTRL2_UARTEN,
		port->port.membase + HW_UARTAPP_CTRL2_SET);

	if (port->clk == NULL || IS_ERR(port->clk)) {
		port->clk = clk_get(NULL, "uart");
		if (port->clk == NULL || IS_ERR(port->clk))
			return -ENODEV;
		port->port.uartclk = clk_get_rate(port->clk);
	}

	if (options)
		uart_parse_options(options, &baud, &parity, &bits, &flow);
	else
		auart_console_get_options(port, &baud, &parity, &bits);
	return uart_set_options(port, co, baud, parity, bits, flow);
}

static struct console auart_console = {
	.name = "ttySP",
	.write = auart_console_write,
	.device = uart_console_device,
	.setup = auart_console_setup,
	.flags = CON_PRINTBUFFER,
	.index = -1,
	.data = &auart_driver,
};

#ifdef CONFIG_MXS_EARLY_CONSOLE
static int __init auart_console_init(void)
{
	register_console(&auart_console);
	return 0;
}

console_initcall(auart_console_init);
#endif

#endif
static struct uart_driver auart_driver = {
	.owner = THIS_MODULE,
	.driver_name = "auart",
	.dev_name = "ttySP",
	.major = MXS_AUART_MAJOR,
	.minor = 0,
	.nr = CONFIG_MXS_AUART_PORTS,
#ifdef CONFIG_SERIAL_MXS_AUART_CONSOLE
	.cons = &auart_console,
#endif
};

static int __devinit mxs_auart_probe(struct platform_device *pdev)
{
	struct mxs_auart_plat_data *plat;
	struct mxs_auart_port *s;
	u32 version;
	int i, ret = 0;
	struct resource *r;

	s = kzalloc(sizeof(struct mxs_auart_port), GFP_KERNEL);
	if (!s) {
		ret = -ENOMEM;
		goto out;
	}

	plat = pdev->dev.platform_data;
	if (plat == NULL) {
		ret = -ENOMEM;
		goto out_free;
	}

	if (plat && plat->clk)
		s->clk = clk_get(NULL, plat->clk);
	else
		s->clk = clk_get(NULL, "uart");
	if (IS_ERR(s->clk)) {
		ret = PTR_ERR(s->clk);
		goto out_free;
	}

	clk_enable(s->clk);

	r = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!r) {
		ret = -ENXIO;
		goto out_free_clk;
	}
	s->port.mapbase = r->start;
	s->port.membase = (void __iomem *)IO_ADDRESS(r->start);
	s->port.ops = &mxs_auart_ops;
	s->port.iotype = UPIO_MEM;
	s->port.line = pdev->id < 0 ? 0 : pdev->id;
	s->port.fifosize = plat->fifo_size;
	s->port.timeout = plat->timeout ? plat->timeout : (HZ / 10);
	s->port.uartclk = clk_get_rate(s->clk);
	s->port.type = PORT_IMX;
	s->port.dev = s->dev = get_device(&pdev->dev);

	s->flags = plat->dma_mode ? MXS_AUART_PORT_DMA_MODE : 0;
	if (s->flags & MXS_AUART_PORT_DMA_MODE)
		s->port.flags |= ASYNC_LOW_LATENCY;

	s->ctrl = 0;
	s->dma_rx_buffer_size = plat->dma_rx_buffer_size;

	for (i = 0; i < ARRAY_SIZE(s->irq); i++) {
		s->irq[i] = platform_get_irq(pdev, i);
		if (s->irq[i] < 0) {
			ret = s->irq[i];
			goto out_free_clk;
		}
	}
	s->port.irq = s->irq[0];

	r = platform_get_resource(pdev, IORESOURCE_DMA, 0);
	if (!r) {
		ret = -ENXIO;
		goto out_free_clk;
	}
	s->dma_rx_chan = r->start;

	r = platform_get_resource(pdev, IORESOURCE_DMA, 1);
	if (!r) {
		ret = -ENXIO;
		goto out_free_clk;
	}
	s->dma_tx_chan = r->start;

	platform_set_drvdata(pdev, s);

	device_init_wakeup(&pdev->dev, 1);

#ifdef CONFIG_SERIAL_MXS_AUART_CONSOLE
	memcpy(&auart_port[pdev->id], s, sizeof(struct mxs_auart_port));
#endif

	ret = uart_add_one_port(&auart_driver, &s->port);
	if (ret)
		goto out_free_clk;

	version = __raw_readl(s->port.membase + HW_UARTAPP_VERSION);
	printk(KERN_INFO "Found APPUART %d.%d.%d\n",
	       (version >> 24) & 0xFF,
	       (version >> 16) & 0xFF, version & 0xFFFF);
	return 0;

out_free_clk:
	if (!IS_ERR(s->clk))
		clk_put(s->clk);
out_free:
	kfree(s);
out:
	return ret;
}

static int __devexit mxs_auart_remove(struct platform_device *pdev)
{
	struct mxs_auart_port *s;

	s = platform_get_drvdata(pdev);
	if (s) {
		put_device(s->dev);
		clk_disable(s->clk);
		clk_put(s->clk);
		uart_remove_one_port(&auart_driver, &s->port);
		kfree(s);
	}
	return 0;
}

static struct platform_driver mxs_auart_driver = {
	.probe = mxs_auart_probe,
	.remove = __devexit_p(mxs_auart_remove),
	.driver = {
		.name = "mxs-auart",
		.owner = THIS_MODULE,
	},
};

static int __init mxs_auart_init(void)
{
	int r;

	r = uart_register_driver(&auart_driver);
	if (r)
		goto out;
	r = platform_driver_register(&mxs_auart_driver);
	if (r)
		goto out_err;
	return 0;
out_err:
	uart_unregister_driver(&auart_driver);
out:
	return r;
}

static void __exit mxs_auart_exit(void)
{
	platform_driver_unregister(&mxs_auart_driver);
	uart_unregister_driver(&auart_driver);
}

module_init(mxs_auart_init)
module_exit(mxs_auart_exit)
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Freescale MXS application uart driver");
