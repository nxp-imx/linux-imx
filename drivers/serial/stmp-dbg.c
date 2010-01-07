/*
 *  Freescale STMP37XX/STMP378X Debug UART driver
 *
 *  Based on drivers/char/serial.c, by Linus Torvalds, Theodore Ts'o.
 *
 *  Copyright 1999 ARM Limited
 *  Copyright (C) 2000 Deep Blue Solutions Ltd.
 *  Modifications for STMP36XX Debug Serial (c) 2005 Sigmatel Inc
 *
 * Copyright 2008-2010 Freescale Semiconductor, Inc. All Rights Reserved.
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
#include <linux/autoconf.h>

#if defined(CONFIG_SERIAL_STMP_DBG_CONSOLE) && defined(CONFIG_MAGIC_SYSRQ)
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

#include <mach/regs-uartdbg.h>

#include <mach/stmp3xxx.h>
#include <mach/platform.h>

#include "stmp-dbg.h"

/* treated as variable unless submitted to open-source */
#define PORT_STMPDBG		100
#define UART_NR			1
#define SERIAL_STMPDBG_MAJOR	204
#define SERIAL_STMPDBG_MINOR	16

#define ISR_PASS_LIMIT		256

#define STMPDBG_DEVID		"Debug UART"


static int force_cd = 1;

static struct uart_driver stmpdbg_reg;

/*
 * We wrap our port structure around the generic uart_port.
 */
struct uart_stmpdbg_port {
	struct uart_port	port;
	struct clk		*clk;
	unsigned int		im;	/* interrupt mask */
	unsigned int		old_status;
	int			suspended;
};


static void stmpdbg_stop_tx(struct uart_port *port)
{
	struct uart_stmpdbg_port *uap = (struct uart_stmpdbg_port *)port;

	uap->im &= ~UART011_TXIM;
	__raw_writel(uap->im, uap->port.membase + UART011_IMSC);
}

static void stmpdbg_start_tx(struct uart_port *port)
{
	struct uart_stmpdbg_port *uap = (struct uart_stmpdbg_port *)port;

	uap->im |= UART011_TXIM;
	__raw_writel(uap->im, uap->port.membase + UART011_IMSC);
}

static void stmpdbg_stop_rx(struct uart_port *port)
{
	struct uart_stmpdbg_port *uap = (struct uart_stmpdbg_port *)port;

	uap->im &= ~(UART011_RXIM|UART011_RTIM|UART011_FEIM|
		     UART011_PEIM|UART011_BEIM|UART011_OEIM);
	__raw_writel(uap->im, uap->port.membase + UART011_IMSC);
}

static void stmpdbg_enable_ms(struct uart_port *port)
{
	struct uart_stmpdbg_port *uap = (struct uart_stmpdbg_port *)port;

	uap->im |= UART011_RIMIM|UART011_CTSMIM|UART011_DCDMIM|UART011_DSRMIM;
	__raw_writel(uap->im, uap->port.membase + UART011_IMSC);
}

static void stmpdbg_rx_chars(struct uart_stmpdbg_port *uap)
{
	struct tty_struct *tty = uap->port.info->port.tty;
	unsigned int status, ch, flag, rsr, max_count = 256;

	status = __raw_readl(uap->port.membase + UART01x_FR);
	while ((status & UART01x_FR_RXFE) == 0 && max_count--) {
#if 0
		if (tty->flip.count >= TTY_FLIPBUF_SIZE) {
			if (tty->low_latency)
				tty_flip_buffer_push(tty);
			/*
			 * If this failed then we will throw away the
			 * bytes but must do so to clear interrupts
			 */
		}
#endif

		ch = __raw_readl(uap->port.membase + UART01x_DR);
		flag = TTY_NORMAL;
		uap->port.icount.rx++;

		/*
		 * Note that the error handling code is
		 * out of the main execution path
		 */
		rsr = __raw_readl(uap->port.membase + UART01x_RSR)
			| UART_DUMMY_RSR_RX;
		if (unlikely(rsr & UART01x_RSR_ANY)) {
			if (rsr & UART01x_RSR_BE) {
				rsr &= ~(UART01x_RSR_FE | UART01x_RSR_PE);
				uap->port.icount.brk++;
				if (uart_handle_break(&uap->port))
					goto ignore_char;
			} else if (rsr & UART01x_RSR_PE)
				uap->port.icount.parity++;
			else if (rsr & UART01x_RSR_FE)
				uap->port.icount.frame++;
			if (rsr & UART01x_RSR_OE)
				uap->port.icount.overrun++;

			rsr &= uap->port.read_status_mask;

			if (rsr & UART01x_RSR_BE)
				flag = TTY_BREAK;
			else if (rsr & UART01x_RSR_PE)
				flag = TTY_PARITY;
			else if (rsr & UART01x_RSR_FE)
				flag = TTY_FRAME;
		}

		if (uart_handle_sysrq_char(&uap->port, ch))
			goto ignore_char;

		uart_insert_char(&uap->port, rsr, UART01x_RSR_OE, ch, flag);

ignore_char:
		status = __raw_readl(uap->port.membase + UART01x_FR);
	}
	tty_flip_buffer_push(tty);
	return;
}

static void stmpdbg_tx_chars(struct uart_stmpdbg_port *uap)
{
	struct circ_buf *xmit = &uap->port.info->xmit;
	int count;

	if (uap->port.x_char) {
		__raw_writel(uap->port.x_char, uap->port.membase + UART01x_DR);
		uap->port.icount.tx++;
		uap->port.x_char = 0;
		return;
	}
	if (uart_circ_empty(xmit) || uart_tx_stopped(&uap->port)) {
		stmpdbg_stop_tx(&uap->port);
		return;
	}

	count = uap->port.fifosize >> 1;
	do {
		__raw_writel(xmit->buf[xmit->tail],
				uap->port.membase + UART01x_DR);
		xmit->tail = (xmit->tail + 1) & (UART_XMIT_SIZE - 1);
		uap->port.icount.tx++;
		if (uart_circ_empty(xmit))
			break;
	} while (--count > 0);

	if (uart_circ_chars_pending(xmit) < WAKEUP_CHARS)
		uart_write_wakeup(&uap->port);

	if (uart_circ_empty(xmit))
		stmpdbg_stop_tx(&uap->port);
}

static void stmpdbg_modem_status(struct uart_stmpdbg_port *uap)
{
	unsigned int status, delta;

	status = __raw_readl(uap->port.membase + UART01x_FR) &
		UART01x_FR_MODEM_ANY;

	delta = status ^ uap->old_status;
	uap->old_status = status;

	if (!delta)
		return;

	if (delta & UART01x_FR_DCD)
		uart_handle_dcd_change(&uap->port, status & UART01x_FR_DCD);

	if (delta & UART01x_FR_DSR)
		uap->port.icount.dsr++;

	if (delta & UART01x_FR_CTS)
		uart_handle_cts_change(&uap->port, status & UART01x_FR_CTS);

	wake_up_interruptible(&uap->port.info->delta_msr_wait);
}

static irqreturn_t stmpdbg_int(int irq, void *dev_id)
{
	struct uart_stmpdbg_port *uap = dev_id;
	unsigned int status, pass_counter = ISR_PASS_LIMIT;
	int handled = 0;

	spin_lock(&uap->port.lock);

	status = __raw_readl(uap->port.membase + UART011_MIS);
	if (status) {
		do {
			__raw_writel(status & ~(UART011_TXIS|UART011_RTIS|
					  UART011_RXIS),
			       uap->port.membase + UART011_ICR);

			if (status & (UART011_RTIS|UART011_RXIS))
				stmpdbg_rx_chars(uap);
			if (status & (UART011_DSRMIS|UART011_DCDMIS|
				      UART011_CTSMIS|UART011_RIMIS))
				stmpdbg_modem_status(uap);
			if (status & UART011_TXIS)
				stmpdbg_tx_chars(uap);

			if (pass_counter-- == 0)
				break;

			status = __raw_readl(uap->port.membase + UART011_MIS);
		} while (status != 0);
		handled = 1;
	}

	spin_unlock(&uap->port.lock);

	return IRQ_RETVAL(handled);
}

static unsigned int stmpdbg_tx_empty(struct uart_port *port)
{
	struct uart_stmpdbg_port *uap = (struct uart_stmpdbg_port *)port;
	unsigned int status = __raw_readl(uap->port.membase + UART01x_FR);
	return status & (UART01x_FR_BUSY|UART01x_FR_TXFF) ? 0 : TIOCSER_TEMT;
}

static unsigned int stmpdbg_get_mctrl(struct uart_port *port)
{
	struct uart_stmpdbg_port *uap = (struct uart_stmpdbg_port *)port;
	unsigned int result = 0;
	unsigned int status = __raw_readl(uap->port.membase + UART01x_FR);

#define TEST_AND_SET_BIT(uartbit, tiocmbit)	do { \
		if (status & uartbit)		\
			result |= tiocmbit;	\
	} while (0)

	TEST_AND_SET_BIT(UART01x_FR_DCD, TIOCM_CAR);
	TEST_AND_SET_BIT(UART01x_FR_DSR, TIOCM_DSR);
	TEST_AND_SET_BIT(UART01x_FR_CTS, TIOCM_CTS);
	TEST_AND_SET_BIT(UART011_FR_RI, TIOCM_RNG);
#undef TEST_AND_SET_BIT
	if (force_cd)
		result |= TIOCM_CAR;
	return result;
}

static void stmpdbg_set_mctrl(struct uart_port *port, unsigned int mctrl)
{
	struct uart_stmpdbg_port *uap = (struct uart_stmpdbg_port *)port;
	unsigned int cr;

	cr = __raw_readl(uap->port.membase + UART011_CR);

#define	TEST_AND_SET_BIT(tiocmbit, uartbit)	do { \
		if (mctrl & tiocmbit)		\
			cr |= uartbit;		\
		else				\
			cr &= ~uartbit;		\
	} while (0)

	TEST_AND_SET_BIT(TIOCM_RTS, UART011_CR_RTS);
	TEST_AND_SET_BIT(TIOCM_DTR, UART011_CR_DTR);
	TEST_AND_SET_BIT(TIOCM_OUT1, UART011_CR_OUT1);
	TEST_AND_SET_BIT(TIOCM_OUT2, UART011_CR_OUT2);
	TEST_AND_SET_BIT(TIOCM_LOOP, UART011_CR_LBE);
#undef TEST_AND_SET_BIT

	__raw_writel(cr, uap->port.membase + UART011_CR);
}

static void stmpdbg_break_ctl(struct uart_port *port, int break_state)
{
	struct uart_stmpdbg_port *uap = (struct uart_stmpdbg_port *)port;
	unsigned long flags;
	unsigned int lcr_h;

	spin_lock_irqsave(&uap->port.lock, flags);
	lcr_h = __raw_readl(uap->port.membase + UART011_LCRH);
	if (break_state == -1)
		lcr_h |= UART01x_LCRH_BRK;
	else
		lcr_h &= ~UART01x_LCRH_BRK;
	__raw_writel(lcr_h, uap->port.membase + UART011_LCRH);
	spin_unlock_irqrestore(&uap->port.lock, flags);
}

static int stmpdbg_startup(struct uart_port *port)
{
	struct uart_stmpdbg_port *uap = (struct uart_stmpdbg_port *)port;
	u32 cr, lcr;
	int retval;

	/*
	 * Allocate the IRQ
	 */
	retval = request_irq(uap->port.irq, stmpdbg_int, 0, STMPDBG_DEVID, uap);
	if (retval)
		return retval;

	__raw_writel(0, uap->port.membase + UART01x_DR); /* wake up the UART */

	__raw_writel(UART011_IFLS_RX4_8|UART011_IFLS_TX4_8,
	       uap->port.membase + UART011_IFLS);

	/*
	 * Provoke TX FIFO interrupt into asserting.
	 */
	cr = UART01x_CR_UARTEN | UART011_CR_RXE | UART011_CR_TXE;
	__raw_writel(cr, uap->port.membase + UART011_CR);

	lcr = __raw_readl(uap->port.membase + UART011_LCRH);
	lcr |= UART01x_LCRH_FEN;
	__raw_writel(lcr, uap->port.membase + UART011_LCRH);

	/*
	 * initialise the old status of the modem signals
	 */
	uap->old_status = __raw_readl(uap->port.membase + UART01x_FR) &
		UART01x_FR_MODEM_ANY;

	/*
	 * Finally, enable interrupts
	 */
	spin_lock_irq(&uap->port.lock);
	uap->im = UART011_RXIM | UART011_RTIM;
	__raw_writel(uap->im, uap->port.membase + UART011_IMSC);
	spin_unlock_irq(&uap->port.lock);

	return 0;
}

static void stmpdbg_shutdown(struct uart_port *port)
{
	struct uart_stmpdbg_port *uap = (struct uart_stmpdbg_port *)port;
	unsigned long val;

	/*
	 * disable all interrupts
	 */
	spin_lock_irq(&uap->port.lock);
	uap->im = 0;
	__raw_writel(uap->im, uap->port.membase + UART011_IMSC);
	__raw_writel(0xffff, uap->port.membase + UART011_ICR);
	spin_unlock_irq(&uap->port.lock);

	free_irq(uap->port.irq, uap);

	/*
	 * disable the port
	 */
	__raw_writel(UART01x_CR_UARTEN | UART011_CR_TXE,
			uap->port.membase + UART011_CR);

	/*
	 * disable break condition and fifos
	 */
	val = __raw_readl(uap->port.membase + UART011_LCRH);
	val &= ~(UART01x_LCRH_BRK | UART01x_LCRH_FEN);
	__raw_writel(val, uap->port.membase + UART011_LCRH);
}

static void
stmpdbg_set_termios(struct uart_port *port, struct ktermios *termios,
		     struct ktermios *old)
{
	unsigned int lcr_h, old_cr;
	unsigned long flags;
	unsigned int baud, quot;

	/*
	 * Ask the core to calculate the divisor for us.
	 */
	baud = uart_get_baud_rate(port, termios, old, 0, port->uartclk/16);
	quot = port->uartclk * 4 / baud;

	switch (termios->c_cflag & CSIZE) {
	case CS5:
		lcr_h = UART01x_LCRH_WLEN_5;
		break;
	case CS6:
		lcr_h = UART01x_LCRH_WLEN_6;
		break;
	case CS7:
		lcr_h = UART01x_LCRH_WLEN_7;
		break;
	default: /* CS8 */
		lcr_h = UART01x_LCRH_WLEN_8;
		break;
	}
	if (termios->c_cflag & CSTOPB)
		lcr_h |= UART01x_LCRH_STP2;
	if (termios->c_cflag & PARENB) {
		lcr_h |= UART01x_LCRH_PEN;
		if (!(termios->c_cflag & PARODD))
			lcr_h |= UART01x_LCRH_EPS;
	}
	lcr_h |= UART01x_LCRH_FEN;

	spin_lock_irqsave(&port->lock, flags);

	/*
	 * Update the per-port timeout.
	 */
	uart_update_timeout(port, termios->c_cflag, baud);

	port->read_status_mask = UART01x_RSR_OE;
	if (termios->c_iflag & INPCK)
		port->read_status_mask |= UART01x_RSR_FE | UART01x_RSR_PE;
	if (termios->c_iflag & (BRKINT | PARMRK))
		port->read_status_mask |= UART01x_RSR_BE;

	/*
	 * Characters to ignore
	 */
	port->ignore_status_mask = 0;
	if (termios->c_iflag & IGNPAR)
		port->ignore_status_mask |= UART01x_RSR_FE | UART01x_RSR_PE;
	if (termios->c_iflag & IGNBRK) {
		port->ignore_status_mask |= UART01x_RSR_BE;
		/*
		 * If we're ignoring parity and break indicators,
		 * ignore overruns too (for real raw support).
		 */
		if (termios->c_iflag & IGNPAR)
			port->ignore_status_mask |= UART01x_RSR_OE;
	}

	/*
	 * Ignore all characters if CREAD is not set.
	 */
	if ((termios->c_cflag & CREAD) == 0)
		port->ignore_status_mask |= UART_DUMMY_RSR_RX;

	if (UART_ENABLE_MS(port, termios->c_cflag))
		stmpdbg_enable_ms(port);

	/* first, disable everything */
	old_cr = __raw_readl(port->membase + UART011_CR);
	__raw_writel(0, port->membase + UART011_CR);

	/* Set baud rate */
	__raw_writel(quot & 0x3f, port->membase + UART011_FBRD);
	__raw_writel(quot >> 6, port->membase + UART011_IBRD);

	/*
	 * ----------v----------v----------v----------v-----
	 * NOTE: MUST BE WRITTEN AFTER UARTLCR_M & UARTLCR_L
	 * ----------^----------^----------^----------^-----
	 */
	__raw_writel(lcr_h, port->membase + UART011_LCRH);
	__raw_writel(old_cr, port->membase + UART011_CR);

	spin_unlock_irqrestore(&port->lock, flags);
}

static const char *stmpdbg_type(struct uart_port *port)
{
	return port->type == PORT_STMPDBG ? STMPDBG_DEVID : NULL;
}


/*
 * Release the memory region(s) being used by 'port'
 */
static void stmpdbg_release_port(struct uart_port *port)
{
	release_mem_region(port->mapbase, UART_PORT_SIZE);
}

/*
 * Request the memory region(s) being used by 'port'
 */
static int stmpdbg_request_port(struct uart_port *port)
{
	return request_mem_region(port->mapbase, UART_PORT_SIZE, STMPDBG_DEVID)
			!= NULL ? 0 : -EBUSY;
}

/*
 * Configure/autoconfigure the port.
 */
static void stmpdbg_config_port(struct uart_port *port, int flags)
{
	if (flags & UART_CONFIG_TYPE) {
		port->type = PORT_STMPDBG;
		stmpdbg_request_port(port);
	}
}

/*
 * verify the new serial_struct (for TIOCSSERIAL).
 */
static int stmpdbg_verify_port(struct uart_port *port,
		struct serial_struct *ser)
{
	int ret = 0;
	if (ser->type != PORT_UNKNOWN && ser->type != PORT_STMPDBG)
		ret = -EINVAL;
	if (ser->irq < 0 || ser->irq >= NR_IRQS)
		ret = -EINVAL;
	if (ser->baud_base < 9600)
		ret = -EINVAL;
	return ret;
}

#ifdef CONFIG_CONSOLE_POLL
/*
 * Console polling routines for writing and reading from the UART while
 * in an interrupt or debug context.
 */

static int stmpdbg_get_poll_char(struct uart_port *port)
{
	struct uart_stmpdbg_port *uap = (struct uart_stmpdbg_port *)port;
	unsigned int status;

	/* Wait until a character arrives. */

	do {
		status = __raw_readl(uap->port.membase + UART01x_FR);
	} while (status & UART01x_FR_RXFE);

	/* Read the character and return it. */

	return __raw_readl(uap->port.membase + UART01x_DR) & 0xff;

}

static void stmpdbg_put_poll_char(struct uart_port *port, unsigned char c)
{
	struct uart_stmpdbg_port *uap = (struct uart_stmpdbg_port *)port;

	/* Wait until the transmit FIFO is empty. */

	while (!(__raw_readl(uap->port.membase + UART01x_FR) & UART011_FR_TXFE))
		barrier();

	/* Transmit the character. */

	__raw_writel(c, uap->port.membase + UART01x_DR);

}

#endif /* CONFIG_CONSOLE_POLL */

static struct uart_ops stmpdbg_pops = {
	.tx_empty	= stmpdbg_tx_empty,
	.set_mctrl	= stmpdbg_set_mctrl,
	.get_mctrl	= stmpdbg_get_mctrl,
	.stop_tx	= stmpdbg_stop_tx,
	.start_tx	= stmpdbg_start_tx,
	.stop_rx	= stmpdbg_stop_rx,
	.enable_ms	= stmpdbg_enable_ms,
	.break_ctl	= stmpdbg_break_ctl,
	.startup	= stmpdbg_startup,
	.shutdown	= stmpdbg_shutdown,
	.set_termios	= stmpdbg_set_termios,
	.type		= stmpdbg_type,
	.release_port	= stmpdbg_release_port,
	.request_port	= stmpdbg_request_port,
	.config_port	= stmpdbg_config_port,
	.verify_port	= stmpdbg_verify_port,
#ifdef CONFIG_CONSOLE_POLL
	.poll_get_char  = stmpdbg_get_poll_char,
	.poll_put_char  = stmpdbg_put_poll_char,
#endif
};

static struct uart_stmpdbg_port stmpdbg_ports[UART_NR] = {
	{
		.port	= {
			/* This *is* the virtual address */
			.membase	= (void *)REGS_UARTDBG_BASE + HW_UARTDBGDR,
			.mapbase	= REGS_UARTDBG_PHYS + HW_UARTDBGDR,
			.iotype		= SERIAL_IO_MEM,
			.irq		= IRQ_DEBUG_UART,
			.fifosize	= 16,
			.ops		= &stmpdbg_pops,
			.flags		= ASYNC_BOOT_AUTOCONF,
			.line		= 0,
			.uartclk	= 24000000,
		},
	}
};

#ifdef CONFIG_SERIAL_STMP_DBG_CONSOLE

static void
stmpdbg_console_write(struct console *co, const char *s, unsigned int count)
{
	struct uart_port *port = &stmpdbg_ports[co->index].port;
	unsigned int status, old_cr;
	int i;

	/*
	 *	First save the CR then disable the interrupts
	 */
	old_cr = UART_GET_CR(port);
	UART_PUT_CR(port, UART01x_CR_UARTEN);

	/*
	 *	Now, do each character
	 */
	for (i = 0; i < count; i++) {
		do {
			status = UART_GET_FR(port);
		} while (!UART_TX_READY(status));
		UART_PUT_CHAR(port, s[i]);
		if (s[i] == '\n') {
			do {
				status = UART_GET_FR(port);
			} while (!UART_TX_READY(status));
			UART_PUT_CHAR(port, '\r');
		}
	}

	/*
	 *	Finally, wait for transmitter to become empty
	 *	and restore the TCR
	 */
	do {
		status = UART_GET_FR(port);
	} while (status & UART01x_FR_BUSY);
	UART_PUT_CR(port, old_cr);
}

static void __init
stmpdbg_console_get_options(struct uart_port *port, int *baud,
			     int *parity, int *bits)
{
	if (UART_GET_CR(port) & UART01x_CR_UARTEN) {
		unsigned int lcr_h, quot;
		lcr_h = UART_GET_LCRH(port);

		*parity = 'n';
		if (lcr_h & UART01x_LCRH_PEN) {
			if (lcr_h & UART01x_LCRH_EPS)
				*parity = 'e';
			else
				*parity = 'o';
		}

		if ((lcr_h & 0x60) == UART01x_LCRH_WLEN_7)
			*bits = 7;
		else
			*bits = 8;

		quot = UART_GET_LCRL(port) | UART_GET_LCRM(port) << 8;
		*baud = port->uartclk / (16 * (quot + 1));
	}
}

static int __init stmpdbg_console_setup(struct console *co, char *options)
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
	if (co->index >= UART_NR)
		co->index = 0;
	port = &stmpdbg_ports[co->index].port;

	if (options)
		uart_parse_options(options, &baud, &parity, &bits, &flow);
	else
		stmpdbg_console_get_options(port, &baud, &parity, &bits);

	return uart_set_options(port, co, baud, parity, bits, flow);
}


static struct console stmpdbg_console = {
	.name		= "ttyAM",
	.write		= stmpdbg_console_write,
	.device		= uart_console_device,
	.setup		= stmpdbg_console_setup,
	.flags		= CON_PRINTBUFFER,
	.index		= -1,
	.data		= &stmpdbg_reg,
};

static int __init stmpdbg_console_init(void)
{
	/*
	 * All port initializations are done statically
	 */
	register_console(&stmpdbg_console);
	return 0;
}
console_initcall(stmpdbg_console_init);

static int __init stmpdbg_late_console_init(void)
{
	if (!(stmpdbg_console.flags & CON_ENABLED))
		register_console(&stmpdbg_console);
	return 0;
}
late_initcall(stmpdbg_late_console_init);

#endif

static struct uart_driver stmpdbg_reg = {
	.owner			= THIS_MODULE,
	.driver_name		= "ttyAM",
	.dev_name		= "ttyAM",
	.major			= SERIAL_STMPDBG_MAJOR,
	.minor			= SERIAL_STMPDBG_MINOR,
	.nr			= UART_NR,
#ifdef CONFIG_SERIAL_STMP_DBG_CONSOLE
	.cons			= &stmpdbg_console,
#endif
};

static int __devinit stmpdbguart_probe(struct platform_device *device)
{
	int ret = 0;
	int i;
	void (*cfg)(int request, int port) = NULL;

	if (device->dev.platform_data)
		cfg = device->dev.platform_data;

	device_init_wakeup(&device->dev, 1);

	for (i = 0; i < UART_NR; i++) {
		stmpdbg_ports[i].clk = clk_get(NULL, "uart");
		if (IS_ERR(stmpdbg_ports[i].clk))
			continue;
		stmpdbg_ports[i].suspended = 0;
		stmpdbg_ports[i].port.dev = &device->dev;
		stmpdbg_ports[i].port.uartclk =
			clk_get_rate(stmpdbg_ports[i].clk) * 1000;
		if (cfg)
			(*cfg)(1, i);
		uart_add_one_port(&stmpdbg_reg, &stmpdbg_ports[i].port);
	}
	return ret;
}

static int __devexit stmpdbguart_remove(struct platform_device *device)
{
	int i;
	void (*cfg)(int request, int port) = NULL;

	if (device->dev.platform_data)
		cfg = device->dev.platform_data;
	for (i = 0; i < UART_NR; i++) {
		clk_put(stmpdbg_ports[i].clk);
		uart_remove_one_port(&stmpdbg_reg, &stmpdbg_ports[0].port);
		if (cfg)
			(*cfg)(0, i);
	}
	return 0;
}

static int stmpdbguart_suspend(struct platform_device *device,
					pm_message_t state)
{
#ifdef CONFIG_PM
	int i;
	int deep_sleep = (stmp37xx_pm_get_target() != PM_SUSPEND_STANDBY);

	for (i = 0; i < UART_NR; i++) {
		if (deep_sleep) {
			uart_suspend_port(&stmpdbg_reg,
				     &stmpdbg_ports[i].port);
			clk_disable(stmpdbg_ports[i].clk);
			stmpdbg_ports[i].suspended = 1;
		}
	}
#endif
	return 0;
}

static int stmpdbguart_resume(struct platform_device *device)
{
	int ret = 0;
#ifdef CONFIG_PM
	int i;

	for (i = 0; i < UART_NR; i++) {
		if (stmpdbg_ports[i].suspended) {
			clk_enable(stmpdbg_ports[i].clk);
			uart_resume_port(&stmpdbg_reg, &stmpdbg_ports[i].port);
		}
		stmpdbg_ports[i].suspended = 0;
	}
#endif
	return ret;
}

static struct platform_driver stmpdbguart_driver = {
	.probe = stmpdbguart_probe,
	.remove = __devexit_p(stmpdbguart_remove),
	.suspend = stmpdbguart_suspend,
	.resume = stmpdbguart_resume,
	.driver = {
		.name = "stmp3xxx-dbguart",
		.owner = THIS_MODULE,
	},
};

static int __init stmpdbg_init(void)
{
	int ret;

	ret = uart_register_driver(&stmpdbg_reg);
	if (ret)
		goto out;

	ret = platform_driver_register(&stmpdbguart_driver);
	if (ret)
		uart_unregister_driver(&stmpdbg_reg);
out:
	return ret;
}

static void __exit stmpdbg_exit(void)
{
	platform_driver_unregister(&stmpdbguart_driver);
	uart_unregister_driver(&stmpdbg_reg);
}

module_init(stmpdbg_init);
module_exit(stmpdbg_exit);
module_param(force_cd, int, 0644);
MODULE_AUTHOR("ARM Ltd/Deep Blue Solutions Ltd/Sigmatel Inc");
MODULE_DESCRIPTION("STMP37xx debug uart");
MODULE_LICENSE("GPL");
