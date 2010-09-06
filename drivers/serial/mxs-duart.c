/*
 * Copyright 2008 Embedded Alley Solutions, Inc All Rights Reserved.
 * Copyright (C) 2009-2010 Freescale Semiconductor, Inc.
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
 */

#if defined(CONFIG_SERIAL_MXS_DUART_CONSOLE) && defined(CONFIG_MAGIC_SYSRQ)
#define SUPPORT_SYSRQ
#endif

#include <linux/module.h>
#include <linux/ioport.h>
#include <linux/init.h>
#include <linux/console.h>
#include <linux/sysrq.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/tty.h>
#include <linux/tty_flip.h>
#include <linux/serial_core.h>
#include <linux/serial.h>
#include <linux/io.h>
#include <linux/irq.h>

#include <mach/hardware.h>
#include <mach/device.h>
#include "regs-duart.h"

/* treated as variable unless submitted to open-source */
#define PORT_DUART		100
#define SERIAL_DUART_MAJOR	204
#define SERIAL_DUART_MINOR	16
#define SERIAL_RX_LIMIT		256
#define ISR_PASS_LIMIT		256

#define DUART_DEVID		"DebugUART"

static int force_cd = 1;
static struct uart_driver duart_drv;

/*
 * We wrap our port structure around the generic uart_port.
 */
struct duart_port {
	struct uart_port port;
	struct clk *clk;
	unsigned int im;	/* interrupt mask */
	unsigned int old_status;
	int suspended;
};

static void duart_stop_tx(struct uart_port *port)
{
	struct duart_port *dp = container_of(port, struct duart_port, port);

	dp->im &= ~BM_UARTDBGIMSC_TXIM;
	__raw_writel(dp->im, dp->port.membase + HW_UARTDBGIMSC);
}

static void duart_start_tx(struct uart_port *port)
{
	struct duart_port *dp = container_of(port, struct duart_port, port);

	dp->im |= BM_UARTDBGIMSC_TXIM;
	__raw_writel(dp->im, dp->port.membase + HW_UARTDBGIMSC);
}

static void duart_stop_rx(struct uart_port *port)
{
	struct duart_port *dp = container_of(port, struct duart_port, port);

	dp->im &= ~(BM_UARTDBGIMSC_OEIM | BM_UARTDBGIMSC_BEIM |
		    BM_UARTDBGIMSC_PEIM | BM_UARTDBGIMSC_FEIM |
		    BM_UARTDBGIMSC_RTIM | BM_UARTDBGIMSC_RXIM);
	__raw_writel(dp->im, dp->port.membase + HW_UARTDBGIMSC);
}

static void duart_enable_ms(struct uart_port *port)
{
	struct duart_port *dp = container_of(port, struct duart_port, port);

	dp->im |= BM_UARTDBGIMSC_RIMIM | BM_UARTDBGIMSC_CTSMIM |
	    BM_UARTDBGIMSC_DCDMIM | BM_UARTDBGIMSC_DSRMIM;
	__raw_writel(dp->im, dp->port.membase + HW_UARTDBGIMSC);
}

static void duart_rx_chars(struct duart_port *dp)
{
	struct tty_struct *tty = dp->port.state->port.tty;
	unsigned int status, ch, flag, rsr, max_count = SERIAL_RX_LIMIT;

	status = __raw_readl(dp->port.membase + HW_UARTDBGFR);
	while ((status & BM_UARTDBGFR_RXFE) == 0 && max_count--) {
		ch = __raw_readl(dp->port.membase + HW_UARTDBGDR);
		flag = TTY_NORMAL;
		dp->port.icount.rx++;

		/*
		 * Note that the error handling code is
		 * out of the main execution path
		 */
		rsr = __raw_readl(dp->port.membase + HW_UARTDBGRSR_ECR);
		if (unlikely(rsr & (BM_UARTDBGRSR_ECR_OE |
				    BM_UARTDBGRSR_ECR_BE |
				    BM_UARTDBGRSR_ECR_PE |
				    BM_UARTDBGRSR_ECR_FE))) {
			if (rsr & BM_UARTDBGRSR_ECR_BE) {
				rsr &= ~(BM_UARTDBGRSR_ECR_FE |
					 BM_UARTDBGRSR_ECR_PE);
				dp->port.icount.brk++;
				if (uart_handle_break(&dp->port))
					goto ignore_char;
			} else if (rsr & BM_UARTDBGRSR_ECR_PE)
				dp->port.icount.parity++;
			else if (rsr & BM_UARTDBGRSR_ECR_FE)
				dp->port.icount.frame++;
			if (rsr & BM_UARTDBGRSR_ECR_OE)
				dp->port.icount.overrun++;

			rsr &= dp->port.read_status_mask;

			if (rsr & BM_UARTDBGRSR_ECR_BE)
				flag = TTY_BREAK;
			else if (rsr & BM_UARTDBGRSR_ECR_PE)
				flag = TTY_PARITY;
			else if (rsr & BM_UARTDBGRSR_ECR_FE)
				flag = TTY_FRAME;
		}

		if (uart_handle_sysrq_char(&dp->port, ch))
			goto ignore_char;

		uart_insert_char(&dp->port, rsr, BM_UARTDBGRSR_ECR_OE, ch,
				 flag);

ignore_char:
		status = __raw_readl(dp->port.membase + HW_UARTDBGFR);
	}
	tty_flip_buffer_push(tty);
	return;
}

static void duart_tx_chars(struct duart_port *dp)
{
	int count;
	struct circ_buf *xmit = &dp->port.state->xmit;

	if (dp->port.x_char) {
		__raw_writel(dp->port.x_char, dp->port.membase + HW_UARTDBGDR);
		dp->port.icount.tx++;
		dp->port.x_char = 0;
		return;
	}
	if (uart_circ_empty(xmit) || uart_tx_stopped(&dp->port)) {
		duart_stop_tx(&dp->port);
		return;
	}

	count = dp->port.fifosize >> 1;
	do {
		__raw_writel(xmit->buf[xmit->tail],
			     dp->port.membase + HW_UARTDBGDR);
		xmit->tail = (xmit->tail + 1) & (UART_XMIT_SIZE - 1);
		dp->port.icount.tx++;
		if (uart_circ_empty(xmit))
			break;
	} while (--count > 0);

	if (uart_circ_chars_pending(xmit) < WAKEUP_CHARS)
		uart_write_wakeup(&dp->port);

	if (uart_circ_empty(xmit))
		duart_stop_tx(&dp->port);
}

static void duart_modem_status(struct duart_port *dp)
{
	unsigned int status, delta;
	status = __raw_readl(dp->port.membase + HW_UARTDBGFR) &
	    (BM_UARTDBGFR_DCD | BM_UARTDBGFR_DSR | BM_UARTDBGFR_CTS);

	delta = status ^ dp->old_status;
	dp->old_status = status;

	if (!delta)
		return;

	if (delta & BM_UARTDBGFR_DCD)
		uart_handle_dcd_change(&dp->port, status & BM_UARTDBGFR_DCD);

	if (delta & BM_UARTDBGFR_DSR)
		dp->port.icount.dsr++;

	if (delta & BM_UARTDBGFR_CTS)
		uart_handle_cts_change(&dp->port, status & BM_UARTDBGFR_CTS);

	wake_up_interruptible(&dp->port.state->port.delta_msr_wait);
}

static irqreturn_t duart_int(int irq, void *dev_id)
{
	int handled = 0;
	struct duart_port *dp = dev_id;
	unsigned int status, pass_counter = ISR_PASS_LIMIT;

	spin_lock(&dp->port.lock);

	status = __raw_readl(dp->port.membase + HW_UARTDBGMIS);
	while (status) {
		handled = 1;

		__raw_writel(status & ~(BM_UARTDBGMIS_TXMIS |
					BM_UARTDBGMIS_RTMIS |
					BM_UARTDBGMIS_RXMIS),
			     dp->port.membase + HW_UARTDBGICR);

		if (status & (BM_UARTDBGMIS_RTMIS | BM_UARTDBGMIS_RXMIS))
			duart_rx_chars(dp);
		if (status & (BM_UARTDBGMIS_DSRMMIS |
			      BM_UARTDBGMIS_DCDMMIS |
			      BM_UARTDBGMIS_CTSMMIS | BM_UARTDBGMIS_RIMMIS))
			duart_modem_status(dp);
		if (status & BM_UARTDBGMIS_TXMIS)
			duart_tx_chars(dp);

		if (pass_counter-- == 0)
			break;

		status = __raw_readl(dp->port.membase + HW_UARTDBGMIS);
	};

	spin_unlock(&dp->port.lock);

	return IRQ_RETVAL(handled);
}

static unsigned int duart_tx_empty(struct uart_port *port)
{
	struct duart_port *dp = (struct duart_port *)port;
	unsigned int status = __raw_readl(dp->port.membase + HW_UARTDBGFR);
	return status & (BM_UARTDBGFR_BUSY | BM_UARTDBGFR_TXFF) ?
	    0 : TIOCSER_TEMT;
}

static unsigned int duart_get_mctrl(struct uart_port *port)
{
	unsigned int result = 0;
	struct duart_port *dp = (struct duart_port *)port;
	unsigned int status = __raw_readl(dp->port.membase + HW_UARTDBGFR);

#define TEST_AND_SET_BIT(uartbit, tiocmbit)	do { \
		if (status & uartbit)		\
			result |= tiocmbit;	\
	} while (0)

	TEST_AND_SET_BIT(BM_UARTDBGFR_DCD, TIOCM_CAR);
	TEST_AND_SET_BIT(BM_UARTDBGFR_DSR, TIOCM_DSR);
	TEST_AND_SET_BIT(BM_UARTDBGFR_CTS, TIOCM_CTS);
	TEST_AND_SET_BIT(BM_UARTDBGFR_RI, TIOCM_RNG);
#undef TEST_AND_SET_BIT
	if (force_cd)
		result |= TIOCM_CAR;
	return result;
}

static void duart_set_mctrl(struct uart_port *port, unsigned int mctrl)
{
	unsigned int cr;
	struct duart_port *dp = (struct duart_port *)port;

	cr = __raw_readl(dp->port.membase + HW_UARTDBGCR);

#define	TEST_AND_SET_BIT(tiocmbit, uartbit)	do { \
		if (mctrl & tiocmbit)		\
			cr |= uartbit;		\
		else				\
			cr &= ~uartbit;		\
	} while (0)

	TEST_AND_SET_BIT(TIOCM_RTS, BM_UARTDBGCR_RTS);
	TEST_AND_SET_BIT(TIOCM_DTR, BM_UARTDBGCR_DTR);
	TEST_AND_SET_BIT(TIOCM_OUT1, BM_UARTDBGCR_OUT1);
	TEST_AND_SET_BIT(TIOCM_OUT2, BM_UARTDBGCR_OUT2);
	TEST_AND_SET_BIT(TIOCM_LOOP, BM_UARTDBGCR_LBE);
#undef TEST_AND_SET_BIT

	__raw_writel(cr, dp->port.membase + HW_UARTDBGCR);
}

static void duart_break_ctl(struct uart_port *port, int break_state)
{
	unsigned long flags;
	unsigned int lcr_h;
	struct duart_port *dp = (struct duart_port *)port;

	spin_lock_irqsave(&dp->port.lock, flags);
	lcr_h = __raw_readl(dp->port.membase + HW_UARTDBGLCR_H);
	if (break_state == -1)
		lcr_h |= BM_UARTDBGLCR_H_BRK;
	else
		lcr_h &= ~BM_UARTDBGLCR_H_BRK;
	__raw_writel(lcr_h, dp->port.membase + HW_UARTDBGLCR_H);
	spin_unlock_irqrestore(&dp->port.lock, flags);
}

static int duart_startup(struct uart_port *port)
{
	u32 cr, lcr;
	int retval;
	struct duart_port *dp = (struct duart_port *)port;

	/*
	 * Allocate the IRQ
	 */
	retval = request_irq(dp->port.irq, duart_int, 0, DUART_DEVID, dp);
	if (retval)
		return retval;

	/* wake up the UART */
	__raw_writel(0, dp->port.membase + HW_UARTDBGDR);

	__raw_writel(BF_UARTDBGIFLS_TXIFLSEL(BV_UARTDBGIFLS_TXIFLSEL__ONE_HALF)
		     |
		     BF_UARTDBGIFLS_RXIFLSEL(BV_UARTDBGIFLS_RXIFLSEL__ONE_HALF),
		     dp->port.membase + HW_UARTDBGIFLS);

	/*
	 * Provoke TX FIFO interrupt into asserting.
	 */
	cr = BM_UARTDBGCR_UARTEN | BM_UARTDBGCR_RXE | BM_UARTDBGCR_TXE;
	__raw_writel(cr, dp->port.membase + HW_UARTDBGCR);

	lcr = __raw_readl(dp->port.membase + HW_UARTDBGLCR_H);
	lcr |= BM_UARTDBGLCR_H_FEN;
	__raw_writel(lcr, dp->port.membase + HW_UARTDBGLCR_H);

	/*
	 * initialise the old status of the modem signals
	 */
	dp->old_status = __raw_readl(dp->port.membase + HW_UARTDBGFR) &
	    (BM_UARTDBGFR_DCD | BM_UARTDBGFR_DSR | BM_UARTDBGFR_CTS);
	/*
	 * Finally, enable interrupts
	 */
	dp->im = BM_UARTDBGIMSC_RXIM | BM_UARTDBGIMSC_RTIM;
	__raw_writel(dp->im, dp->port.membase + HW_UARTDBGIMSC);

	return 0;
}

static void duart_shutdown(struct uart_port *port)
{
	unsigned long flags;
	unsigned int val;
	struct duart_port *dp = (struct duart_port *)port;

	/*
	 * disable all interrupts
	 */
	spin_lock_irqsave(&dp->port.lock, flags);
	dp->im = 0;
	__raw_writel(dp->im, dp->port.membase + HW_UARTDBGIMSC);
	__raw_writel(0xffff, dp->port.membase + HW_UARTDBGICR);
	spin_unlock_irqrestore(&dp->port.lock, flags);

	free_irq(dp->port.irq, dp);

	/*
	 * disable the port
	 */
	__raw_writel(BM_UARTDBGCR_UARTEN | BM_UARTDBGCR_TXE,
		     dp->port.membase + HW_UARTDBGCR);
	/*
	 * disable break condition and fifos
	 */
	val = __raw_readl(dp->port.membase + HW_UARTDBGLCR_H);
	val &= ~(BM_UARTDBGLCR_H_BRK | BM_UARTDBGLCR_H_FEN);
	__raw_writel(val, dp->port.membase + HW_UARTDBGLCR_H);
}

static void
duart_set_termios(struct uart_port *port, struct ktermios *termios,
		  struct ktermios *old)
{
	unsigned int lcr_h, old_cr;
	unsigned long flags;
	unsigned int baud, quot;

	/*
	 * Ask the core to calculate the divisor for us.
	 */
	baud = uart_get_baud_rate(port, termios, old, 0, port->uartclk / 16);
	quot = (port->uartclk << 2) / baud;

	switch (termios->c_cflag & CSIZE) {
	case CS5:
		lcr_h = BF_UARTDBGLCR_H_WLEN(0);
		break;
	case CS6:
		lcr_h = BF_UARTDBGLCR_H_WLEN(1);
		break;
	case CS7:
		lcr_h = BF_UARTDBGLCR_H_WLEN(2);
		break;
	default:		/* CS8 */
		lcr_h = BF_UARTDBGLCR_H_WLEN(3);
		break;
	}
	if (termios->c_cflag & CSTOPB)
		lcr_h |= BM_UARTDBGLCR_H_STP2;
	if (termios->c_cflag & PARENB) {
		lcr_h |= BM_UARTDBGLCR_H_PEN;
		if (!(termios->c_cflag & PARODD))
			lcr_h |= BM_UARTDBGLCR_H_EPS;
	}
	lcr_h |= BM_UARTDBGLCR_H_FEN;

	spin_lock_irqsave(&port->lock, flags);

	/*
	 * Update the per-port timeout.
	 */
	uart_update_timeout(port, termios->c_cflag, baud);

	port->read_status_mask = BM_UARTDBGRSR_ECR_OE;
	if (termios->c_iflag & INPCK)
		port->read_status_mask |= BM_UARTDBGRSR_ECR_FE |
		    BM_UARTDBGRSR_ECR_PE;
	if (termios->c_iflag & (BRKINT | PARMRK))
		port->read_status_mask |= BM_UARTDBGRSR_ECR_BE;

	/*
	 * Characters to ignore
	 */
	port->ignore_status_mask = 0;
	if (termios->c_iflag & IGNPAR)
		port->ignore_status_mask |= BM_UARTDBGRSR_ECR_FE |
		    BM_UARTDBGRSR_ECR_PE;
	if (termios->c_iflag & IGNBRK) {
		port->ignore_status_mask |= BM_UARTDBGRSR_ECR_BE;
		/*
		 * If we're ignoring parity and break indicators,
		 * ignore overruns too (for real raw support).
		 */
		if (termios->c_iflag & IGNPAR)
			port->ignore_status_mask |= BM_UARTDBGRSR_ECR_OE;
	}

	if (UART_ENABLE_MS(port, termios->c_cflag))
		duart_enable_ms(port);

	/* first, disable everything */
	old_cr = __raw_readl(port->membase + HW_UARTDBGCR);
	__raw_writel(0, port->membase + HW_UARTDBGCR);

	/* Set baud rate */
	__raw_writel(quot & 0x3f, port->membase + HW_UARTDBGFBRD);
	__raw_writel(quot >> 6, port->membase + HW_UARTDBGIBRD);
	/*
	 * ----------v----------v----------v----------v-----
	 * NOTE: MUST BE WRITTEN AFTER UARTLCR_M & UARTLCR_L
	 * ----------^----------^----------^----------^-----
	 */
	__raw_writel(lcr_h, port->membase + HW_UARTDBGLCR_H);
	__raw_writel(old_cr, port->membase + HW_UARTDBGCR);

	spin_unlock_irqrestore(&port->lock, flags);
}

static const char *duart_type(struct uart_port *port)
{
	return port->type == PORT_DUART ? DUART_DEVID : NULL;
}

/*
 * Release the memory region(s) being used by 'port'
 */
static void duart_release_port(struct uart_port *port)
{
	release_mem_region(port->mapbase, PAGE_SIZE);
}

/*
 * Request the memory region(s) being used by 'port'
 */
static int duart_request_port(struct uart_port *port)
{
	return request_mem_region(port->mapbase, PAGE_SIZE, DUART_DEVID)
	    != NULL ? 0 : -EBUSY;
}

/*
 * Configure/autoconfigure the port.
 */
static void duart_config_port(struct uart_port *port, int flags)
{
	if (flags & UART_CONFIG_TYPE) {
		port->type = PORT_DUART;
		duart_request_port(port);
	}
}

/*
 * verify the new serial_struct (for TIOCSSERIAL).
 */
static int duart_verify_port(struct uart_port *port, struct serial_struct *ser)
{
	int ret = 0;
	if (ser->type != PORT_UNKNOWN && ser->type != PORT_DUART)
		ret = -EINVAL;
	if (ser->irq < 0 || ser->irq >= NR_IRQS)
		ret = -EINVAL;
	if (ser->baud_base < 9600)
		ret = -EINVAL;
	return ret;
}

static struct uart_ops duart_pops = {
	.tx_empty = duart_tx_empty,
	.set_mctrl = duart_set_mctrl,
	.get_mctrl = duart_get_mctrl,
	.stop_tx = duart_stop_tx,
	.start_tx = duart_start_tx,
	.stop_rx = duart_stop_rx,
	.enable_ms = duart_enable_ms,
	.break_ctl = duart_break_ctl,
	.startup = duart_startup,
	.shutdown = duart_shutdown,
	.set_termios = duart_set_termios,
	.type = duart_type,
	.release_port = duart_release_port,
	.request_port = duart_request_port,
	.config_port = duart_config_port,
	.verify_port = duart_verify_port,
};

static struct duart_port duart_port = {
	.port = {
		 .iotype = SERIAL_IO_MEM,
#ifdef CONFIG_MXS_EARLY_CONSOLE
		 .membase = MXS_DEBUG_CONSOLE_VIRT,
		 .mapbase = MXS_DEBUG_CONSOLE_PHYS,
#endif
		 .fifosize = 16,
		 .ops = &duart_pops,
		 .flags = ASYNC_BOOT_AUTOCONF,
		 .line = 0,
		 },
};

#ifdef CONFIG_SERIAL_MXS_DUART_CONSOLE

static void
duart_console_write(struct console *co, const char *s, unsigned int count)
{
	struct uart_port *port = &duart_port.port;
	unsigned int status, old_cr;
	int i;
	/*
	 *      First save the CR then disable the interrupts
	 */
	old_cr = __raw_readl(port->membase + HW_UARTDBGCR);
	__raw_writel(BM_UARTDBGCR_UARTEN | BM_UARTDBGCR_TXE,
		     port->membase + HW_UARTDBGCR);
	/*
	 *      Now, do each character
	 */
	for (i = 0; i < count; i++) {
		do {
			status = __raw_readl(port->membase + HW_UARTDBGFR);
		} while (status & BM_UARTDBGFR_TXFF);

		__raw_writel(s[i], port->membase + HW_UARTDBGDR);
		if (s[i] == '\n') {
			do {
				status = __raw_readl(port->membase +
						     HW_UARTDBGFR);
			} while (status & BM_UARTDBGFR_TXFF);
			__raw_writel('\r', port->membase + HW_UARTDBGDR);
		}
	}

	/*
	 *      Finally, wait for transmitter to become empty
	 *      and restore the TCR
	 */
	do {
		status = __raw_readl(port->membase + HW_UARTDBGFR);
	} while (status & BM_UARTDBGFR_BUSY);
	__raw_writel(old_cr, port->membase + HW_UARTDBGCR);
}

static void __init
duart_console_get_options(struct uart_port *port, int *baud,
			  int *parity, int *bits)
{
	if (__raw_readl(port->membase + HW_UARTDBGCR) & BM_UARTDBGCR_UARTEN) {
		unsigned int lcr_h, quot;
		lcr_h = __raw_readl(port->membase + HW_UARTDBGLCR_H);

		*parity = 'n';
		if (lcr_h & BM_UARTDBGLCR_H_PEN) {
			if (lcr_h & BM_UARTDBGLCR_H_EPS)
				*parity = 'e';
			else
				*parity = 'o';
		}

		if ((lcr_h & BM_UARTDBGLCR_H_WLEN) == BF_UARTDBGLCR_H_WLEN(2))
			*bits = 7;
		else
			*bits = 8;

		quot = (__raw_readl(port->membase + HW_UARTDBGFBRD) & 0x3F) |
		    __raw_readl(port->membase + HW_UARTDBGIBRD) << 6;
		if (quot == 0)
			quot = 1;
		*baud = (port->uartclk << 2) / quot;
	}
}

static int __init duart_console_setup(struct console *co, char *options)
{
	struct uart_port *port;
	int baud = 115200;
	int bits = 8;
	int parity = 'n';
	int flow = 'n';
	/*
	 * Check whether an invalid uart number has been specified, and
	 * if so, search for the first available port that does have
	 * console support.
	 */
	if (co->index)
		return -EINVAL;

	port = &duart_port.port;

	if (duart_port.clk == NULL || IS_ERR(duart_port.clk)) {
		duart_port.clk = clk_get(NULL, "uart");
		if (duart_port.clk == NULL || IS_ERR(duart_port.clk))
			return -ENODEV;
		duart_port.port.uartclk = clk_get_rate(duart_port.clk);
	}

	if (options)
		uart_parse_options(options, &baud, &parity, &bits, &flow);
	else
		duart_console_get_options(port, &baud, &parity, &bits);
	return uart_set_options(port, co, baud, parity, bits, flow);
}

static struct console duart_console = {
	.name = "ttyAM",
	.write = duart_console_write,
	.device = uart_console_device,
	.setup = duart_console_setup,
	.flags = CON_PRINTBUFFER,
	.index = -1,
	.data = &duart_drv,
};

#ifdef CONFIG_MXS_EARLY_CONSOLE
static int __init duart_console_init(void)
{
	register_console(&duart_console);
	return 0;
}

console_initcall(duart_console_init);
#endif

#endif

static struct uart_driver duart_drv = {
	.owner = THIS_MODULE,
	.driver_name = "ttyAM",
	.dev_name = "ttyAM",
	.major = SERIAL_DUART_MAJOR,
	.minor = SERIAL_DUART_MINOR,
	.nr = 1,
#ifdef CONFIG_SERIAL_MXS_DUART_CONSOLE
	.cons = &duart_console,
#endif
};

static int __devinit duart_probe(struct platform_device *pdev)
{
	struct resource *res;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res)
		return -ENOMEM;
	/*
	 * Will use mapbase and membase here if !CONFIG_MXS_EARLY_CONSOLE,
	 * or use the overridden values later if CONFIG_MXS_EARLY_CONSOLE
	 */
	duart_port.port.mapbase = res->start;
	duart_port.port.membase =
	    (unsigned char __iomem *)IO_ADDRESS(res->start);

	duart_port.port.irq = platform_get_irq(pdev, 0);
	if (duart_port.port.irq < 0)
		return -EINVAL;
	device_init_wakeup(&pdev->dev, 1);

	duart_port.clk = clk_get(NULL, "uart");
	if (duart_port.clk == NULL || IS_ERR(duart_port.clk))
		return -ENODEV;
	duart_port.suspended = 0;
	duart_port.port.dev = &pdev->dev;
	duart_port.port.uartclk = clk_get_rate(duart_port.clk);
	uart_add_one_port(&duart_drv, &duart_port.port);
	return 0;
}

static int __devexit duart_remove(struct platform_device *pdev)
{
	clk_put(duart_port.clk);
	uart_remove_one_port(&duart_drv, &duart_port.port);
	return 0;
}

#ifdef CONFIG_PM
static int duart_suspend(struct platform_device *pdev,
		pm_message_t state)
{
	int ret = 0;
	if (!duart_port.suspended) {
		ret = uart_suspend_port(&duart_drv, &duart_port.port);
		if (!ret)
			duart_port.suspended = 1;
	}
	return ret;
}

static int duart_resume(struct platform_device *pdev,
		pm_message_t state)
{
	int ret = 0;
	if (duart_port.suspended) {
		ret = uart_resume_port(&duart_drv, &duart_port.port);
		if (!ret)
			duart_port.suspended = 0;
	}
	return ret;
}
#else
#define	duart_suspend	NULL
#define	duart_resume	NULL
#endif

static struct platform_driver duart_driver = {
	.probe = duart_probe,
	.remove = __devexit_p(duart_remove),
	.suspend = duart_suspend,
	.resume = duart_resume,
	.driver = {
		   .name = "mxs-duart",
		   .owner = THIS_MODULE,
		   },
};

static int __init duart_init(void)
{
	int ret;
	ret = uart_register_driver(&duart_drv);
	if (ret)
		return ret;

	ret = platform_driver_register(&duart_driver);
	if (ret)
		uart_unregister_driver(&duart_drv);

	return ret;
}

static void __exit duart_exit(void)
{
	platform_driver_unregister(&duart_driver);
	uart_unregister_driver(&duart_drv);
}

module_init(duart_init);
module_exit(duart_exit);
module_param(force_cd, int, 0644);
MODULE_AUTHOR("ARM Ltd/Deep Blue Solutions Ltd/Freescale Inc");
MODULE_DESCRIPTION("i.MXS debug uart");
MODULE_LICENSE("GPL");
