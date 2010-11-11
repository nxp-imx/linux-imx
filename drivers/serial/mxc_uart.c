/*
 * Copyright 2004-2010 Freescale Semiconductor, Inc. All Rights Reserved.
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
 * @file drivers/serial/mxc_uart.c
 *
 * @brief Driver for the Freescale Semiconductor MXC serial ports based on
 * drivers/char/serial.c, by Linus Torvalds, Theodore Ts'o.
 *
 * @ingroup UART
 */

/*
 * Include Files
 */
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/tty.h>
#include <linux/string.h>
#include <linux/ioport.h>
#include <linux/init.h>
#include <linux/serial.h>
#include <linux/console.h>
#include <linux/platform_device.h>
#include <linux/sysrq.h>
#include <linux/dma-mapping.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <asm/irq.h>
#include <asm/dma.h>
#include <asm/div64.h>
#include <mach/mxc_uart.h>

#if defined(CONFIG_SERIAL_MXC_CONSOLE) && defined(CONFIG_MAGIC_SYSRQ)
#define SUPPORT_SYSRQ
#endif
#define SERIAL_MXC_MAJOR        207
#define SERIAL_MXC_MINOR        16
#define MXC_ISR_PASS_LIMIT      256
#define UART_CREAD_BIT          256

#define MXC_UART_NR		8

/* IRDA minimum pulse duration in micro seconds */
#define MIN_PULSE_DUR           2
/*
 * Transmit DMA buffer size is set to 1024 bytes, this is limited
 * by UART_XMIT_SIZE.
 */
#define TXDMA_BUFF_SIZE         UART_XMIT_SIZE
/*
 * Receive DMA sub-buffer size
 */
#define RXDMA_BUFF_SIZE         128

/*!
 * This structure is used to store the information for DMA data transfer.
 */
typedef struct {
	/*!
	 * Holds the read channel number.
	 */
	int rd_channel;
	/*!
	 * Holds the write channel number.
	 */
	int wr_channel;
	/*!
	 * UART Transmit Event ID
	 */
	int tx_event_id;
	/*!
	 * UART Receive Event ID
	 */
	int rx_event_id;
	/*!
	 * DMA Transmit tasklet
	 */
	struct tasklet_struct dma_tx_tasklet;
	/*!
	 * Flag indicates if the channel is in use
	 */
	int dma_txchnl_inuse;
} dma_info;

/*!
 * This is used to indicate if we want echo cancellation in the Irda mode.
 */
static int echo_cancel;
extern void gpio_uart_active(int port, int no_irda);
extern void gpio_uart_inactive(int port, int no_irda);
extern void config_uartdma_event(int port);

static uart_mxc_port *mxc_ports[MXC_UART_NR];

/*!
 * This array holds the DMA channel information for each MXC UART
 */
static dma_info dma_list[MXC_UART_NR];

/*!
 * This function is called by the core driver to stop UART transmission.
 * This might be due to the TTY layer indicating that the user wants to stop
 * transmission.
 *
 * @param   port       the port structure for the UART passed in by the core
 *                     driver
 */
static void mxcuart_stop_tx(struct uart_port *port)
{
	uart_mxc_port *umxc = (uart_mxc_port *) port;
	volatile unsigned int cr1;

	cr1 = readl(port->membase + MXC_UARTUCR1);
	/* Disable Transmitter rdy interrupt */
	if (umxc->dma_enabled == 1) {
		cr1 &= ~MXC_UARTUCR1_TXDMAEN;
	} else {
		cr1 &= ~MXC_UARTUCR1_TRDYEN;
	}
	writel(cr1, port->membase + MXC_UARTUCR1);
}

/*!
 * DMA Transmit tasklet method is scheduled on completion of a DMA transmit
 * to send out any more data that is available in the UART xmit buffer.
 *
 * @param   arg  driver private data
 */
static void dma_tx_do_tasklet(unsigned long arg)
{
	uart_mxc_port *umxc = (uart_mxc_port *) arg;
	struct circ_buf *xmit = &umxc->port.state->xmit;
	mxc_dma_requestbuf_t writechnl_request;
	int tx_num;
	unsigned long flags;

	spin_lock_irqsave(&umxc->port.lock, flags);
	tx_num = uart_circ_chars_pending(xmit);
	if (tx_num > 0) {
		if (xmit->tail > xmit->head) {
			memcpy(umxc->tx_buf, xmit->buf + xmit->tail,
			       UART_XMIT_SIZE - xmit->tail);
			memcpy(umxc->tx_buf + (UART_XMIT_SIZE - xmit->tail),
			       xmit->buf, xmit->head);
		} else {
			memcpy(umxc->tx_buf, xmit->buf + xmit->tail, tx_num);
		}
		umxc->tx_handle = dma_map_single(umxc->port.dev, umxc->tx_buf,
						 TXDMA_BUFF_SIZE,
						 DMA_TO_DEVICE);

		writechnl_request.dst_addr = umxc->port.mapbase + MXC_UARTUTXD;
		writechnl_request.src_addr = umxc->tx_handle;
		writechnl_request.num_of_bytes = tx_num;

		if ((mxc_dma_config(dma_list[umxc->port.line].wr_channel,
				    &writechnl_request, 1,
				    MXC_DMA_MODE_WRITE)) == 0) {
			mxc_dma_enable(dma_list[umxc->port.line].wr_channel);
		}
	} else {
		/* No more data available in the xmit queue, clear the flag */
		dma_list[umxc->port.line].dma_txchnl_inuse = 0;
	}
	spin_unlock_irqrestore(&umxc->port.lock, flags);
}

/*!
 * DMA Write callback is called by the SDMA controller after it has sent out all
 * the data from the user buffer. This function updates the xmit buffer pointers.
 *
 * @param   arg   driver private data
 * @param   error any DMA error
 * @param   count amount of data that was transferred
 */
static void mxcuart_dma_writecallback(void *arg, int error, unsigned int count)
{
	uart_mxc_port *umxc = arg;
	struct circ_buf *xmit = &umxc->port.state->xmit;
	int tx_num;

	if (error != MXC_DMA_TRANSFER_ERROR) {
		tx_num = count;
		umxc->port.icount.tx += tx_num;
		xmit->tail = (xmit->tail + tx_num) & (UART_XMIT_SIZE - 1);
	}

	dma_unmap_single(umxc->port.dev, umxc->tx_handle, TXDMA_BUFF_SIZE,
			 DMA_TO_DEVICE);
	tx_num = uart_circ_chars_pending(xmit);
	/* Schedule a tasklet to send out the pending characters */
	if (tx_num > 0) {
		tasklet_schedule(&dma_list[umxc->port.line].dma_tx_tasklet);
	} else {
		dma_list[umxc->port.line].dma_txchnl_inuse = 0;
	}
	if (tx_num < WAKEUP_CHARS) {
		uart_write_wakeup(&umxc->port);
	}
}

/*!
 * This function is called by the core driver to start transmitting characters.
 * This function enables the transmit interrupts.
 *
 * @param   port       the port structure for the UART passed in by the core
 *                     driver
 */
static void mxcuart_start_tx(struct uart_port *port)
{
	uart_mxc_port *umxc = (uart_mxc_port *) port;
	struct circ_buf *xmit = &umxc->port.state->xmit;
	volatile unsigned int cr1;
	mxc_dma_requestbuf_t writechnl_request;
	int tx_num;

	cr1 = readl(port->membase + MXC_UARTUCR1);
	/* Enable Transmitter rdy interrupt */
	if (umxc->dma_enabled == 1) {
		/*
		 * If the channel is in use then return immediately and use
		 * the dma_tx tasklet to transfer queued data when current DMA
		 * transfer is complete
		 */
		if (dma_list[umxc->port.line].dma_txchnl_inuse == 1) {
			return;
		}
		tx_num = uart_circ_chars_pending(xmit);
		if (tx_num > 0) {
			dma_list[umxc->port.line].dma_txchnl_inuse = 1;
			if (xmit->tail > xmit->head) {
				memcpy(umxc->tx_buf, xmit->buf + xmit->tail,
				       UART_XMIT_SIZE - xmit->tail);
				memcpy(umxc->tx_buf +
				       (UART_XMIT_SIZE - xmit->tail), xmit->buf,
				       xmit->head);
			} else {
				memcpy(umxc->tx_buf, xmit->buf + xmit->tail,
				       tx_num);
			}
			umxc->tx_handle =
			    dma_map_single(umxc->port.dev, umxc->tx_buf,
					   TXDMA_BUFF_SIZE, DMA_TO_DEVICE);

			writechnl_request.dst_addr =
			    umxc->port.mapbase + MXC_UARTUTXD;
			writechnl_request.src_addr = umxc->tx_handle;
			writechnl_request.num_of_bytes = tx_num;
			if ((mxc_dma_config
			     (dma_list[umxc->port.line].wr_channel,
			      &writechnl_request, 1,
			      MXC_DMA_MODE_WRITE)) == 0) {
				mxc_dma_enable(dma_list[umxc->port.line].
					       wr_channel);
			}
			cr1 |= MXC_UARTUCR1_TXDMAEN;
		}
	} else {
		cr1 |= MXC_UARTUCR1_TRDYEN;
	}
	writel(cr1, port->membase + MXC_UARTUCR1);
}

/*!
 * This function is called by the core driver to stop receiving characters; the
 * port is in the process of being closed.
 *
 * @param   port   the port structure for the UART passed in by the core driver
 */
static void mxcuart_stop_rx(struct uart_port *port)
{
	uart_mxc_port *umxc = (uart_mxc_port *) port;
	volatile unsigned int cr1;

	cr1 = readl(port->membase + MXC_UARTUCR1);
	if (umxc->dma_enabled == 1) {
		cr1 &= ~MXC_UARTUCR1_RXDMAEN;
	} else {
		cr1 &= ~MXC_UARTUCR1_RRDYEN;
	}
	writel(cr1, port->membase + MXC_UARTUCR1);
}

/*!
 * This function is called by the core driver to enable the modem status
 * interrupts. If the port is configured to be in DTE mode then it enables the
 * DCDDELT and RIDELT interrupts in addition to the DTRDEN interrupt. The RTSDEN
 * interrupt is enabled only for interrupt-driven hardware flow control.
 *
 * @param   port   the port structure for the UART passed in by the core driver
 */
static void mxcuart_enable_ms(struct uart_port *port)
{
	uart_mxc_port *umxc = (uart_mxc_port *) port;
	volatile unsigned int cr1, cr3;

	/*
	 * RTS interrupt is enabled only if we are using interrupt-driven
	 * software controlled hardware flow control
	 */
	if (umxc->hardware_flow == 0) {
		cr1 = readl(umxc->port.membase + MXC_UARTUCR1);
		cr1 |= MXC_UARTUCR1_RTSDEN;
		writel(cr1, umxc->port.membase + MXC_UARTUCR1);
	}
	cr3 = readl(umxc->port.membase + MXC_UARTUCR3);
	cr3 |= MXC_UARTUCR3_DTRDEN;
	if (umxc->mode == MODE_DTE) {
		cr3 |= MXC_UARTUCR3_DCD | MXC_UARTUCR3_RI;
	}
	writel(cr3, umxc->port.membase + MXC_UARTUCR3);
}

/*!
 * This function is called from the interrupt service routine if the status bit
 * indicates that the receive fifo data level is above the set threshold. The
 * function reads the character and queues them into the TTY layers read
 * buffer. The function also looks for break characters, parity and framing
 * errors in the received character and sets the appropriate flag in the TTY
 * receive buffer.
 *
 * @param   umxc   the MXC UART port structure, this includes the \b uart_port
 *                 structure and other members that are specific to MXC UARTs
 */
static void mxcuart_rx_chars(uart_mxc_port *umxc)
{
	volatile unsigned int ch, sr2;
	unsigned int status, flag, max_count = 256;

	sr2 = readl(umxc->port.membase + MXC_UARTUSR2);
	while (((sr2 & MXC_UARTUSR2_RDR) == 1) && (max_count-- > 0)) {
		ch = readl(umxc->port.membase + MXC_UARTURXD);

		flag = TTY_NORMAL;
		status = ch | UART_CREAD_BIT;
		ch &= 0xFF;	/* Clear the upper bits */
		umxc->port.icount.rx++;

		/*
		 * Check to see if there is an  error in the received
		 * character. Perform the appropriate actions based on the
		 * error bit that was set.
		 */
		if (status & MXC_UARTURXD_ERR) {
			if (status & MXC_UARTURXD_BRK) {
				/*
				 * Clear the frame and parity error bits
				 * as these always get set on receiving a
				 * break character
				 */
				status &= ~(MXC_UARTURXD_FRMERR |
					    MXC_UARTURXD_PRERR);
				umxc->port.icount.brk++;
				if (uart_handle_break(&umxc->port)) {
					goto ignore_char;
				}
			} else if (status & MXC_UARTURXD_FRMERR) {
				umxc->port.icount.frame++;
			} else if (status & MXC_UARTURXD_PRERR) {
				umxc->port.icount.parity++;
			}
			if (status & MXC_UARTURXD_OVRRUN) {
				umxc->port.icount.overrun++;
			}

			status &= umxc->port.read_status_mask;

			if (status & MXC_UARTURXD_BRK) {
				flag = TTY_BREAK;
			} else if (status & MXC_UARTURXD_FRMERR) {
				flag = TTY_FRAME;
			} else if (status & MXC_UARTURXD_PRERR) {
				flag = TTY_PARITY;
			}
		}

		if (uart_handle_sysrq_char(&umxc->port, ch)) {
			goto ignore_char;
		}

		uart_insert_char(&umxc->port, status, MXC_UARTURXD_OVRRUN, ch,
				 flag);
	      ignore_char:
		sr2 = readl(umxc->port.membase + MXC_UARTUSR2);
	}
	tty_flip_buffer_push(umxc->port.state->port.tty);
}

/*!
 * This function is called from the interrupt service routine if the status bit
 * indicates that the transmit fifo is emptied below its set threshold and
 * requires data. The function pulls characters from the TTY layers write
 * buffer and writes it out to the UART transmit fifo.
 *
 * @param   umxc   the MXC UART port structure, this includes the \b uart_port
 *                 structure and other members that are specific to MXC UARTs
 */
static void mxcuart_tx_chars(uart_mxc_port *umxc)
{
	struct circ_buf *xmit = &umxc->port.state->xmit;
	int count;

	/*
	 * Transmit the XON/XOFF character if required
	 */
	if (umxc->port.x_char) {
		writel(umxc->port.x_char, umxc->port.membase + MXC_UARTUTXD);
		umxc->port.icount.tx++;
		umxc->port.x_char = 0;
		return;
	}

	/*
	 * Check to see if there is any data to be sent and that the
	 * port has not been currently stopped by anything.
	 */
	if (uart_circ_empty(xmit) || uart_tx_stopped(&umxc->port)) {
		mxcuart_stop_tx(&umxc->port);
		return;
	}

	count = umxc->port.fifosize - umxc->tx_threshold;
	do {
		writel(xmit->buf[xmit->tail],
		       umxc->port.membase + MXC_UARTUTXD);
		xmit->tail = (xmit->tail + 1) & (UART_XMIT_SIZE - 1);
		umxc->port.icount.tx++;
		if (uart_circ_empty(xmit)) {
			break;
		}
	} while (--count > 0);

	/*
	 * Check to see if we have flushed enough characters to ask for more
	 * to be sent to us, if so, we notify the user space that we can
	 * accept more data
	 */
	if (uart_circ_chars_pending(xmit) < WAKEUP_CHARS) {
		uart_write_wakeup(&umxc->port);
	}

	if (uart_circ_empty(xmit)) {
		mxcuart_stop_tx(&umxc->port);
	}
}

/*!
 * This function is called from the interrupt service routine if there is a
 * change in the modem signals. This function handles these signal changes and
 * also clears the appropriate status register bits.
 *
 * @param   umxc   the MXC UART port structure, this includes the \b uart_port
 *                 structure and other members that are specific to MXC UARTs
 * @param   sr1    contents of status register 1
 * @param   sr2    contents of status register 2
 */
static void mxcuart_modem_status(uart_mxc_port *umxc, unsigned int sr1,
				 unsigned int sr2)
{
	if (umxc->mode == MODE_DTE) {
		if (sr2 & MXC_UARTUSR2_DCDDELT) {
			uart_handle_dcd_change(&umxc->port,
					       !(sr2 & MXC_UARTUSR2_DCDIN));
		}
		if (sr2 & MXC_UARTUSR2_RIDELT) {
			umxc->port.icount.rng++;
		}
	}
	if (sr1 & MXC_UARTUSR1_DTRD) {
		umxc->port.icount.dsr++;
	}
	if ((umxc->hardware_flow == 0) && (sr1 & MXC_UARTUSR1_RTSD)) {
		uart_handle_cts_change(&umxc->port, sr1 & MXC_UARTUSR1_RTSS);
	}

	wake_up_interruptible(&umxc->port.state->port.delta_msr_wait);
}

/*!
 * Interrupt service routine registered to handle the muxed ANDed interrupts.
 * This routine is registered only in the case where the UART interrupts are
 * muxed.
 *
 * @param   irq    the interrupt number
 * @param   dev_id driver private data
 *
 * @return  The function returns \b IRQ_RETVAL(1) if interrupt was handled,
 *          returns \b IRQ_RETVAL(0) if the interrupt was not handled.
 *          \b IRQ_RETVAL is defined in \b include/linux/interrupt.h.
 */
static irqreturn_t mxcuart_int(int irq, void *dev_id)
{
	uart_mxc_port *umxc = dev_id;
	volatile unsigned int sr1, sr2, cr1, cr;
	unsigned int pass_counter = MXC_ISR_PASS_LIMIT;
	unsigned int term_cond = 0;
	int handled = 0;

	sr1 = readl(umxc->port.membase + MXC_UARTUSR1);
	sr2 = readl(umxc->port.membase + MXC_UARTUSR2);
	cr1 = readl(umxc->port.membase + MXC_UARTUCR1);

	do {
		/* Clear the bits that triggered the interrupt */
		writel(sr1, umxc->port.membase + MXC_UARTUSR1);
		writel(sr2, umxc->port.membase + MXC_UARTUSR2);
		/*
		 * Read if there is data available
		 */
		if (sr2 & MXC_UARTUSR2_RDR) {
			mxcuart_rx_chars(umxc);
		}

		if ((sr1 & (MXC_UARTUSR1_RTSD | MXC_UARTUSR1_DTRD)) ||
		    (sr2 & (MXC_UARTUSR2_DCDDELT | MXC_UARTUSR2_RIDELT))) {
			mxcuart_modem_status(umxc, sr1, sr2);
		}

		/*
		 * Send data if there is data to be sent
		 */
		if ((cr1 & MXC_UARTUCR1_TRDYEN) && (sr1 & MXC_UARTUSR1_TRDY)) {
			/* Echo cancellation for IRDA Transmit chars */
			if (umxc->ir_mode == IRDA && echo_cancel) {
				/* Disable the receiver */
				cr = readl(umxc->port.membase + MXC_UARTUCR2);
				cr &= ~MXC_UARTUCR2_RXEN;
				writel(cr, umxc->port.membase + MXC_UARTUCR2);
				/* Enable Transmit complete intr to reenable RX */
				cr = readl(umxc->port.membase + MXC_UARTUCR4);
				cr |= MXC_UARTUCR4_TCEN;
				writel(cr, umxc->port.membase + MXC_UARTUCR4);
			}
			mxcuart_tx_chars(umxc);
		}

		if (pass_counter-- == 0) {
			break;
		}

		sr1 = readl(umxc->port.membase + MXC_UARTUSR1);
		sr2 = readl(umxc->port.membase + MXC_UARTUSR2);

		/* Is the transmit complete to reenable the receiver? */
		if (umxc->ir_mode == IRDA && echo_cancel) {
			if (sr2 & MXC_UARTUSR2_TXDC) {
				cr = readl(umxc->port.membase + MXC_UARTUCR2);
				cr |= MXC_UARTUCR2_RXEN;
				writel(cr, umxc->port.membase + MXC_UARTUCR2);
				/* Disable the Transmit complete interrupt bit */
				cr = readl(umxc->port.membase + MXC_UARTUCR4);
				cr &= ~MXC_UARTUCR4_TCEN;
				writel(cr, umxc->port.membase + MXC_UARTUCR4);
			}
		}

		/*
		 * If there is no data to send or receive and if there is no
		 * change in the modem status signals then quit the routine
		 */
		term_cond = sr1 & (MXC_UARTUSR1_RTSD | MXC_UARTUSR1_DTRD);
		term_cond |= sr2 & (MXC_UARTUSR2_RDR | MXC_UARTUSR2_DCDDELT);
		term_cond |= !(sr2 & MXC_UARTUSR2_TXFE);
	} while (term_cond > 0);

	handled = 1;
	return IRQ_RETVAL(handled);
}

/*!
 * Interrupt service routine registered to handle the transmit interrupts. This
 * routine is registered only in the case where the UART interrupts are not
 * muxed.
 *
 * @param   irq    the interrupt number
 * @param   dev_id driver private data
 *
 * @return  The function returns \b IRQ_RETVAL(1) if interrupt was handled,
 *          returns \b IRQ_RETVAL(0) if the interrupt was not handled.
 *          \b IRQ_RETVAL is defined in include/linux/interrupt.h.
 */
static irqreturn_t mxcuart_tx_int(int irq, void *dev_id)
{
	uart_mxc_port *umxc = dev_id;
	int handled = 0;
	volatile unsigned int sr2, cr;

	/* Echo cancellation for IRDA Transmit chars */
	if (umxc->ir_mode == IRDA && echo_cancel) {
		/* Disable the receiver */
		cr = readl(umxc->port.membase + MXC_UARTUCR2);
		cr &= ~MXC_UARTUCR2_RXEN;
		writel(cr, umxc->port.membase + MXC_UARTUCR2);
		/* Enable Transmit complete to reenable receiver */
		cr = readl(umxc->port.membase + MXC_UARTUCR4);
		cr |= MXC_UARTUCR4_TCEN;
		writel(cr, umxc->port.membase + MXC_UARTUCR4);
	}

	mxcuart_tx_chars(umxc);

	/* Is the transmit complete to reenable the receiver? */
	if (umxc->ir_mode == IRDA && echo_cancel) {
		sr2 = readl(umxc->port.membase + MXC_UARTUSR2);
		if (sr2 & MXC_UARTUSR2_TXDC) {
			cr = readl(umxc->port.membase + MXC_UARTUCR2);
			cr |= MXC_UARTUCR2_RXEN;
			writel(cr, umxc->port.membase + MXC_UARTUCR2);
			/* Disable the Transmit complete interrupt bit */
			cr = readl(umxc->port.membase + MXC_UARTUCR4);
			cr &= ~MXC_UARTUCR4_TCEN;
			writel(cr, umxc->port.membase + MXC_UARTUCR4);
		}
	}

	handled = 1;

	return IRQ_RETVAL(handled);
}

/*!
 * Interrupt service routine registered to handle the receive interrupts. This
 * routine is registered only in the case where the UART interrupts are not
 * muxed.
 *
 * @param   irq    the interrupt number
 * @param   dev_id driver private data
 *
 * @return  The function returns \b IRQ_RETVAL(1) if interrupt was handled,
 *          returns \b IRQ_RETVAL(0) if the interrupt was not handled.
 *          \b IRQ_RETVAL is defined in include/linux/interrupt.h.
 */
static irqreturn_t mxcuart_rx_int(int irq, void *dev_id)
{
	uart_mxc_port *umxc = dev_id;
	int handled = 0;

	/* Clear the aging timer bit */
	writel(MXC_UARTUSR1_AGTIM, umxc->port.membase + MXC_UARTUSR1);
	mxcuart_rx_chars(umxc);
	handled = 1;

	return IRQ_RETVAL(handled);
}

/*!
 * Interrupt service routine registered to handle the master interrupts. This
 * routine is registered only in the case where the UART interrupts are not
 * muxed.
 *
 * @param   irq    the interrupt number
 * @param   dev_id driver private data
 *
 * @return  The function returns \b IRQ_RETVAL(1) if interrupt was handled,
 *          returns \b IRQ_RETVAL(0) if the interrupt was not handled.
 *          \b IRQ_RETVAL is defined in include/linux/interrupt.h.
 */
static irqreturn_t mxcuart_mint_int(int irq, void *dev_id)
{
	uart_mxc_port *umxc = dev_id;
	int handled = 0;
	volatile unsigned int sr1, sr2;

	sr1 = readl(umxc->port.membase + MXC_UARTUSR1);
	sr2 = readl(umxc->port.membase + MXC_UARTUSR2);
	/* Clear the modem status interrupt bits */
	writel(MXC_UARTUSR1_RTSD | MXC_UARTUSR1_DTRD,
	       umxc->port.membase + MXC_UARTUSR1);
	writel(MXC_UARTUSR2_DCDDELT | MXC_UARTUSR2_RIDELT,
	       umxc->port.membase + MXC_UARTUSR2);
	mxcuart_modem_status(umxc, sr1, sr2);
	handled = 1;

	return IRQ_RETVAL(handled);
}

/*!
 * This function is called by the core driver to test whether the transmitter
 * fifo and shift register for the UART port are empty.
 *
 * @param   port   the port structure for the UART passed in by the core driver
 *
 * @return  The function returns TIOCSER_TEMT if it is empty, else returns 0.
 */
static unsigned int mxcuart_tx_empty(struct uart_port *port)
{
	volatile unsigned int sr2;
	unsigned long flags;

	spin_lock_irqsave(&port->lock, flags);
	sr2 = readl(port->membase + MXC_UARTUSR2);
	spin_unlock_irqrestore(&port->lock, flags);

	return sr2 & MXC_UARTUSR2_TXDC ? TIOCSER_TEMT : 0;
}

/*!
 * This function is called by the core driver to get the current status of the
 * modem input signals. The state of the output signals is not collected.
 *
 * @param   port   the port structure for the UART passed in by the core driver
 *
 * @return  The function returns an integer that contains the ORed value of the
 *          status of all the modem input signals or error.
 */
static unsigned int mxcuart_get_mctrl(struct uart_port *port)
{
	uart_mxc_port *umxc = (uart_mxc_port *) port;
	unsigned int result = 0;
	volatile unsigned int sr1, sr2;

	sr1 = readl(umxc->port.membase + MXC_UARTUSR1);
	sr2 = readl(umxc->port.membase + MXC_UARTUSR2);

	if (sr1 & MXC_UARTUSR1_RTSS) {
		result |= TIOCM_CTS;
	}
	if (umxc->mode == MODE_DTE) {
		if (!(sr2 & MXC_UARTUSR2_DCDIN)) {
			result |= TIOCM_CAR;
		}
		if (!(sr2 & MXC_UARTUSR2_RIIN)) {
			result |= TIOCM_RI;
		}
	}
	return result;
}

/*!
 * This function is called by the core driver to set the state of the modem
 * control lines.
 *
 * @param   port   the port structure for the UART passed in by the core driver
 * @param   mctrl  the state that the modem control lines should be changed to
 */
static void mxcuart_set_mctrl(struct uart_port *port, unsigned int mctrl)
{
	uart_mxc_port *umxc = (uart_mxc_port *) port;
	volatile unsigned int cr2 = 0, cr3 = 0, uts = 0;

	cr2 = readl(port->membase + MXC_UARTUCR2);
	cr3 = readl(port->membase + MXC_UARTUCR3);
	uts = readl(port->membase + MXC_UARTUTS);

	if (mctrl & TIOCM_RTS) {
		/*
		 * Return to hardware-driven hardware flow control if the
		 * option is enabled
		 */
		if (umxc->hardware_flow == 1) {
			cr2 |= MXC_UARTUCR2_CTSC;
		} else {
			cr2 |= MXC_UARTUCR2_CTS;
			cr2 &= ~MXC_UARTUCR2_CTSC;
		}
	} else {
		cr2 &= ~(MXC_UARTUCR2_CTS | MXC_UARTUCR2_CTSC);
	}
	writel(cr2, port->membase + MXC_UARTUCR2);

	if (mctrl & TIOCM_DTR) {
		cr3 |= MXC_UARTUCR3_DSR;
	} else {
		cr3 &= ~MXC_UARTUCR3_DSR;
	}
	writel(cr3, port->membase + MXC_UARTUCR3);

	if (mctrl & TIOCM_LOOP) {
		if (umxc->ir_mode == IRDA) {
			echo_cancel = 0;
		} else {
			uts |= MXC_UARTUTS_LOOP;
		}
	} else {
		if (umxc->ir_mode == IRDA) {
			echo_cancel = 1;
		} else {
			uts &= ~MXC_UARTUTS_LOOP;
		}
	}
	writel(uts, port->membase + MXC_UARTUTS);
}

/*!
 * This function is called by the core driver to control the transmission of
 * the break signal. If break_state is non-zero, the break signal is
 * transmitted, the signal is terminated when another call is made with
 * break_state set to 0.
 *
 * @param   port          the port structure for the UART passed in by the core
 *                        driver
 * @param   break_state   the requested state of the break signal
 */
static void mxcuart_break_ctl(struct uart_port *port, int break_state)
{
	volatile unsigned int cr1;
	unsigned long flags;

	spin_lock_irqsave(&port->lock, flags);
	cr1 = readl(port->membase + MXC_UARTUCR1);
	if (break_state == -1) {
		cr1 |= MXC_UARTUCR1_SNDBRK;
	} else {
		cr1 &= ~MXC_UARTUCR1_SNDBRK;
	}
	writel(cr1, port->membase + MXC_UARTUCR1);
	spin_unlock_irqrestore(&port->lock, flags);
}

/*!
 * The read DMA callback, this method is called when the DMA buffer has received its
 * data. This functions copies the data to the tty buffer and updates the tty buffer
 * pointers. It also queues the DMA buffer back to the DMA system.
 *
 * @param   arg   driver private data
 * @param   error any DMA error
 * @param   cnt   amount of data that was transferred
 */
static void mxcuart_dmaread_callback(void *arg, int error, unsigned int cnt)
{
	uart_mxc_port *umxc = arg;
	struct tty_struct *tty = umxc->port.state->port.tty;
	int buff_id, flip_cnt, num_bufs;
	mxc_dma_requestbuf_t readchnl_request;
	mxc_uart_rxdmamap *rx_buf_elem = NULL;
	unsigned int sr1, sr2;
	char flag;

	num_bufs = umxc->dma_rxbuf_size / RXDMA_BUFF_SIZE;
	/* Clear the aging timer bit */
	writel(MXC_UARTUSR1_AGTIM, umxc->port.membase + MXC_UARTUSR1);

	buff_id = umxc->dma_rxbuf_id;
	flag = TTY_NORMAL;

	umxc->dma_rxbuf_id += 1;
	if (umxc->dma_rxbuf_id >= num_bufs) {
		umxc->dma_rxbuf_id = 0;
	}

	rx_buf_elem = (mxc_uart_rxdmamap *) (umxc->rx_dmamap + buff_id);

	if (error == MXC_DMA_TRANSFER_ERROR) {

		sr1 = __raw_readl(umxc->port.membase + MXC_UARTUSR1);
		sr2 = __raw_readl(umxc->port.membase + MXC_UARTUSR2);

		if (sr2 & MXC_UARTUSR2_BRCD) {
			umxc->port.icount.brk++;
			if (uart_handle_break(&umxc->port)) {
				goto drop_data;
			}
		} else if (sr1 & MXC_UARTUSR1_PARITYERR) {
			umxc->port.icount.parity++;
		} else if (sr1 & MXC_UARTUSR1_FRAMERR) {
			umxc->port.icount.frame++;
		} else if (sr2 & MXC_UARTUSR2_ORE) {
			umxc->port.icount.overrun++;

		}

		if (umxc->port.read_status_mask & MXC_UARTURXD_BRK) {
			if (sr2 & MXC_UARTUSR2_BRCD)
				flag = TTY_BREAK;
		} else if (umxc->port.read_status_mask & MXC_UARTURXD_PRERR) {
			if (sr1 & MXC_UARTUSR1_PARITYERR)
				flag = TTY_PARITY;
		} else if (umxc->port.read_status_mask & MXC_UARTURXD_FRMERR) {
			if (sr1 & MXC_UARTUSR1_FRAMERR)
				flag = TTY_FRAME;
		} else if (umxc->port.read_status_mask & MXC_UARTURXD_OVRRUN) {
			if (sr2 & MXC_UARTUSR2_ORE)
				flag = TTY_OVERRUN;
		}
/* By default clearing all error bits in status reg */
		__raw_writel((MXC_UARTUSR2_BRCD | MXC_UARTUSR2_ORE),
			     umxc->port.membase + MXC_UARTUSR2);
		__raw_writel((MXC_UARTUSR1_PARITYERR | MXC_UARTUSR1_FRAMERR),
			     umxc->port.membase + MXC_UARTUSR1);
	}

	flip_cnt = tty_buffer_request_room(tty, cnt);

	/* Check for space availability in the TTY Flip buffer */
	if (flip_cnt <= 0) {
		goto drop_data;
	}
	umxc->port.icount.rx += flip_cnt;

	tty_insert_flip_string(tty, rx_buf_elem->rx_buf, flip_cnt);

	if (flag != TTY_NORMAL) {
		tty_insert_flip_char(tty, 0, flag);
	}

	tty_flip_buffer_push(tty);
	umxc->port.state->port.tty->real_raw = 1;

      drop_data:
	readchnl_request.src_addr = umxc->port.mapbase;
	readchnl_request.dst_addr = rx_buf_elem->rx_handle;
	readchnl_request.num_of_bytes = RXDMA_BUFF_SIZE;
	mxc_dma_config(dma_list[umxc->port.line].rd_channel, &readchnl_request,
		       1, MXC_DMA_MODE_READ);
	mxc_dma_enable(dma_list[umxc->port.line].rd_channel);
}

/*!
 * Allocates DMA read and write channels, creates DMA read and write buffers and
 * sets the channel specific parameters.
 *
 * @param   d_info the structure that holds all the DMA information for a
 *                 particular MXC UART
 * @param   umxc   the MXC UART port structure, this includes the \b uart_port
 *                 structure and other members that are specific to MXC UARTs
 *
 * @return  The function returns 0 on success and a non-zero value on failure.
 */
static int mxcuart_initdma(dma_info *d_info, uart_mxc_port *umxc)
{
	int ret = 0, rxbufs, i, j;
	mxc_dma_requestbuf_t *readchnl_reqelem;
	mxc_uart_rxdmamap *rx_buf_elem;

	/* Request for the read and write channels */
	d_info->rd_channel = mxc_dma_request(umxc->dma_rx_id, "MXC UART Read");
	if (d_info->rd_channel < 0) {
		printk(KERN_ERR "MXC UART: Cannot allocate DMA read channel\n");
		return -1;
	} else {
		d_info->wr_channel =
		    mxc_dma_request(umxc->dma_tx_id, "MXC UART Write");
		if (d_info->wr_channel < 0) {
			mxc_dma_free(d_info->rd_channel);
			printk(KERN_ERR
			       "MXC UART: Cannot allocate DMA write channel\n");
			return -1;
		}
	}

	/* Allocate the DMA Transmit Buffer */
	umxc->tx_buf = kmalloc(TXDMA_BUFF_SIZE, GFP_KERNEL);
	if (umxc->tx_buf == NULL) {
		ret = -1;
		goto err_dma_tx_buff;
	}
	rxbufs = umxc->dma_rxbuf_size / RXDMA_BUFF_SIZE;
	/* Allocate the DMA Virtual Receive Buffer */
	umxc->rx_dmamap = kmalloc(rxbufs * sizeof(mxc_uart_rxdmamap), GFP_KERNEL);
	if (umxc->rx_dmamap == NULL) {
		ret = -1;
		goto err_dma_rx_buff;
	}

	/* Allocate the DMA Receive Request structures */
	readchnl_reqelem = kmalloc(rxbufs * sizeof(mxc_dma_requestbuf_t),
		     GFP_KERNEL);
	if (readchnl_reqelem == NULL) {
		ret = -1;
		goto err_request;
	}

	for (i = 0; i < rxbufs; i++) {
		rx_buf_elem = (mxc_uart_rxdmamap *) (umxc->rx_dmamap + i);
		rx_buf_elem->rx_buf =
		    dma_alloc_coherent(NULL, RXDMA_BUFF_SIZE,
				       &rx_buf_elem->rx_handle, GFP_DMA);
		if (rx_buf_elem->rx_buf == NULL) {
			for (j = 0; j < i; j++) {
				rx_buf_elem =
				    (mxc_uart_rxdmamap *) (umxc->rx_dmamap + j);
				dma_free_coherent(NULL, RXDMA_BUFF_SIZE,
						  rx_buf_elem->rx_buf,
						  rx_buf_elem->rx_handle);
			}
			ret = -1;
			goto cleanup;
		}
	}

	umxc->dma_rxbuf_id = 0;
	/* Setup the DMA read request structures */
	for (i = 0; i < rxbufs; i++) {
		rx_buf_elem = (mxc_uart_rxdmamap *) (umxc->rx_dmamap + i);
		(readchnl_reqelem + i)->src_addr = umxc->port.mapbase;
		(readchnl_reqelem + i)->dst_addr = rx_buf_elem->rx_handle;
		(readchnl_reqelem + i)->num_of_bytes = RXDMA_BUFF_SIZE;
	}
	mxc_dma_config(d_info->rd_channel, readchnl_reqelem, rxbufs,
		       MXC_DMA_MODE_READ);
	mxc_dma_callback_set(d_info->rd_channel, mxcuart_dmaread_callback,
			     umxc);
	mxc_dma_callback_set(d_info->wr_channel, mxcuart_dma_writecallback,
			     umxc);

	/* Start the read channel */
	mxc_dma_enable(d_info->rd_channel);
	kfree(readchnl_reqelem);
	tasklet_init(&d_info->dma_tx_tasklet, dma_tx_do_tasklet,
		     (unsigned long)umxc);
	d_info->dma_txchnl_inuse = 0;
	return ret;
      cleanup:
	kfree(readchnl_reqelem);
      err_request:
	kfree(umxc->rx_dmamap);
      err_dma_rx_buff:
	kfree(umxc->tx_buf);
      err_dma_tx_buff:
	mxc_dma_free(d_info->rd_channel);
	mxc_dma_free(d_info->wr_channel);

	return ret;
}

/*!
 * Stops DMA and frees the DMA resources
 *
 * @param   d_info the structure that holds all the DMA information for a
 *                 particular MXC UART
 * @param   umxc   the MXC UART port structure, this includes the \b uart_port
 *                 structure and other members that are specific to MXC UARTs
 */
static void mxcuart_freedma(dma_info *d_info, uart_mxc_port *umxc)
{
	int i, rxbufs;
	mxc_uart_rxdmamap *rx_buf_elem;

	rxbufs = umxc->dma_rxbuf_size / RXDMA_BUFF_SIZE;

	for (i = 0; i < rxbufs; i++) {
		rx_buf_elem = (mxc_uart_rxdmamap *) (umxc->rx_dmamap + i);
		dma_free_coherent(NULL, RXDMA_BUFF_SIZE,
				  rx_buf_elem->rx_buf, rx_buf_elem->rx_handle);
	}
	kfree(umxc->rx_dmamap);
	kfree(umxc->tx_buf);
	mxc_dma_free(d_info->rd_channel);
	mxc_dma_free(d_info->wr_channel);
}

/*!
 * This function is called to free the interrupts.
 *
 * @param   umxc   the MXC UART port structure, this includes the \b uart_port
 *                 structure and other members that are specific to MXC UARTs
 */
static void mxcuart_free_interrupts(uart_mxc_port *umxc)
{
	free_irq(umxc->port.irq, umxc);
	if (umxc->ints_muxed == 0) {
		free_irq(umxc->irqs[0], umxc);
		free_irq(umxc->irqs[1], umxc);
	}
}

/*!
 * Calculate and set the UART port clock value
 *
 * @param   umxc     the MXC UART port structure, this includes the \b uart_port
 *                   structure and other members that are specific to MXC UARTs
 * @param   per_clk  peripheral clock coming into the MXC UART module
 * @param   req_baud current baudrate requested
 * @param   div      returns the reference frequency divider value
 */
static void mxcuart_set_ref_freq(uart_mxc_port *umxc, unsigned long per_clk,
				 unsigned int req_baud, int *div)
{
	unsigned int d = 1;

	/*
	 * Choose the smallest possible prescaler to maximize
	 * the chance of using integer scaling.  Ensure that
	 * the calculation won't overflow.  Limit the denom
	 * to 15 bits since a 16-bit denom doesn't work.
	 */
	if (req_baud < (1 << (31 - (4 + 15))))
		d = per_clk / (req_baud << (4 + 15)) + 1;

	umxc->port.uartclk = per_clk / d;

	/*
	 * Set the ONEMS register that is used by IR special case bit and
	 * the Escape character detect logic
	 */
	writel(umxc->port.uartclk / 1000, umxc->port.membase + MXC_UARTONEMS);
	*div = d;
}

/*!
 * This function is called by the core driver to initialize the low-level
 * driver. The function grabs the interrupt resources and registers its
 * interrupt service routines. It then initializes the IOMUX registers to
 * configure the pins for UART signals and finally initializes the various
 * UART registers and enables the port for reception.
 *
 * @param   port   the port structure for the UART passed in by the core driver
 *
 * @return  The function returns 0 on success and a non-zero value on failure.
 */
static int mxcuart_startup(struct uart_port *port)
{
	uart_mxc_port *umxc = (uart_mxc_port *) port;
	int retval;
	volatile unsigned int cr, cr1 = 0, cr2 = 0, ufcr = 0;

	/*
	 * Some UARTs need separate registrations for the interrupts as
	 * they do not take the muxed interrupt output to the ARM core
	 */
	if (umxc->ints_muxed == 1) {
		retval = request_irq(umxc->port.irq, mxcuart_int, 0,
				     "mxcintuart", umxc);
		if (retval != 0) {
			return retval;
		}
	} else {
		retval = request_irq(umxc->port.irq, mxcuart_tx_int,
				     0, "mxcintuart", umxc);
		if (retval != 0) {
			return retval;
		} else {
			retval = request_irq(umxc->irqs[0], mxcuart_rx_int,
					     0, "mxcintuart", umxc);
			if (retval != 0) {
				free_irq(umxc->port.irq, umxc);
				return retval;
			} else {
				retval =
				    request_irq(umxc->irqs[1], mxcuart_mint_int,
						0, "mxcintuart", umxc);
				if (retval != 0) {
					free_irq(umxc->port.irq, umxc);
					free_irq(umxc->irqs[0], umxc);
					return retval;
				}
			}
		}
	}

	/* Initialize the DMA if we need SDMA data transfer */
	if (umxc->dma_enabled == 1) {
		retval = mxcuart_initdma(dma_list + umxc->port.line, umxc);
		if (retval != 0) {
			printk
			    (KERN_ERR
			     "MXC UART: Failed to initialize DMA for UART %d\n",
			     umxc->port.line);
			mxcuart_free_interrupts(umxc);
			return retval;
		}
		/* Configure the GPR register to receive SDMA events */
		config_uartdma_event(umxc->port.line);
	}

	/*
	 * Clear Status Registers 1 and 2
	 */
	writel(0xFFFF, umxc->port.membase + MXC_UARTUSR1);
	writel(0xFFFF, umxc->port.membase + MXC_UARTUSR2);

	/* Configure the IOMUX for the UART */
	gpio_uart_active(umxc->port.line, umxc->ir_mode);

	/*
	 * Set the transceiver invert bits if required
	 */
	if (umxc->ir_mode == IRDA) {
		echo_cancel = 1;
		writel(umxc->ir_rx_inv | MXC_UARTUCR4_IRSC, umxc->port.membase
		       + MXC_UARTUCR4);
		writel(umxc->rxd_mux | umxc->ir_tx_inv,
		       umxc->port.membase + MXC_UARTUCR3);
	} else {
		writel(umxc->rxd_mux, umxc->port.membase + MXC_UARTUCR3);
	}

	/*
	 * Initialize UCR1,2 and UFCR registers
	 */
	if (umxc->dma_enabled == 1) {
		cr2 = (MXC_UARTUCR2_TXEN | MXC_UARTUCR2_RXEN);
	} else {
		cr2 =
		    (MXC_UARTUCR2_ATEN | MXC_UARTUCR2_TXEN | MXC_UARTUCR2_RXEN);
	}

	writel(cr2, umxc->port.membase + MXC_UARTUCR2);
	/* Wait till we are out of software reset */
	do {
		cr = readl(umxc->port.membase + MXC_UARTUCR2);
	} while (!(cr & MXC_UARTUCR2_SRST));

	if (umxc->mode == MODE_DTE) {
		ufcr |= ((umxc->tx_threshold << MXC_UARTUFCR_TXTL_OFFSET) |
			 MXC_UARTUFCR_DCEDTE | MXC_UARTUFCR_RFDIV | umxc->
			 rx_threshold);
	} else {
		ufcr |= ((umxc->tx_threshold << MXC_UARTUFCR_TXTL_OFFSET) |
			 MXC_UARTUFCR_RFDIV | umxc->rx_threshold);
	}
	writel(ufcr, umxc->port.membase + MXC_UARTUFCR);

	/*
	 * Finally enable the UART and the Receive interrupts
	 */
	if (umxc->ir_mode == IRDA) {
		cr1 |= MXC_UARTUCR1_IREN;
	}
	if (umxc->dma_enabled == 1) {
		cr1 |= (MXC_UARTUCR1_RXDMAEN | MXC_UARTUCR1_ATDMAEN |
			MXC_UARTUCR1_UARTEN);
	} else {
		cr1 |= (MXC_UARTUCR1_RRDYEN | MXC_UARTUCR1_UARTEN);
	}
	writel(cr1, umxc->port.membase + MXC_UARTUCR1);

	return 0;
}

/*!
 * This function is called by the core driver for the low-level driver to free
 * its resources. The function frees all its interrupts and disables the UART.
 *
 * @param   port   the port structure for the UART passed in by the core driver
 */
static void mxcuart_shutdown(struct uart_port *port)
{
	uart_mxc_port *umxc = (uart_mxc_port *) port;

	/* Disable the IOMUX for the UART */
	gpio_uart_inactive(umxc->port.line, umxc->ir_mode);
	mxcuart_free_interrupts(umxc);
	/* Disable all interrupts, port and break condition */
	writel(0, umxc->port.membase + MXC_UARTUCR1);
	writel(0, umxc->port.membase + MXC_UARTUCR3);
	if (umxc->dma_enabled == 1) {
		mxcuart_freedma(dma_list + umxc->port.line, umxc);
	}
}

/*!
 * This function is called by the core driver to change the UART parameters,
 * including baudrate, word length, parity, stop bits. The function also updates
 * the port structures mask registers to indicate the types of events the user is
 * interested in receiving.
 *
 * @param   port    the port structure for the UART passed in by the core driver
 * @param   termios the desired termios settings
 * @param   old     old termios
 */
static void mxcuart_set_termios(struct uart_port *port,
				struct ktermios *termios, struct ktermios *old)
{
	uart_mxc_port *umxc = (uart_mxc_port *) port;
	volatile unsigned int cr4 = 0, cr2 = 0, ufcr;
	u_int num, denom, baud;
	u_int cr2_mask;		/* Used to add the changes to CR2 */
	unsigned long flags, per_clk;
	int div;

	cr2_mask = ~(MXC_UARTUCR2_IRTS | MXC_UARTUCR2_CTSC | MXC_UARTUCR2_PREN |
		     MXC_UARTUCR2_PROE | MXC_UARTUCR2_STPB | MXC_UARTUCR2_WS);

	per_clk = clk_get_rate(umxc->clk);

	/*
	 * Ask the core to get the baudrate, if requested baudrate is not
	 * between max and min, then either use the baudrate in old termios
	 * setting. If it's still invalid, we try 9600 baud.
	 */
	baud = uart_get_baud_rate(&umxc->port, termios, old, 0, per_clk / 16);
	/* Set the Reference frequency divider */
	mxcuart_set_ref_freq(umxc, per_clk, baud, &div);

	/* Byte size, default is 8-bit mode */
	switch (termios->c_cflag & CSIZE) {
	case CS7:
		cr2 = 0;
		break;
	default:
		cr2 = MXC_UARTUCR2_WS;
		break;
	}
	/* Check to see if we need 2 Stop bits */
	if (termios->c_cflag & CSTOPB) {
		cr2 |= MXC_UARTUCR2_STPB;
	}

	/* Check to see if we need Parity checking */
	if (termios->c_cflag & PARENB) {
		cr2 |= MXC_UARTUCR2_PREN;
		if (termios->c_cflag & PARODD) {
			cr2 |= MXC_UARTUCR2_PROE;
		}
	}
	spin_lock_irqsave(&umxc->port.lock, flags);

	ufcr = readl(umxc->port.membase + MXC_UARTUFCR);
	ufcr = (ufcr & (~MXC_UARTUFCR_RFDIV_MASK)) |
	    ((6 - div) << MXC_UARTUFCR_RFDIV_OFFSET);
	writel(ufcr, umxc->port.membase + MXC_UARTUFCR);

	/*
	 * Update the per-port timeout
	 */
	uart_update_timeout(&umxc->port, termios->c_cflag, baud);

	umxc->port.read_status_mask = MXC_UARTURXD_OVRRUN;
	/*
	 * Enable appropriate events to be passed to the TTY layer
	 */
	if (termios->c_iflag & INPCK) {
		umxc->port.read_status_mask |= MXC_UARTURXD_FRMERR |
		    MXC_UARTURXD_PRERR;
	}
	if (termios->c_iflag & (BRKINT | PARMRK)) {
		umxc->port.read_status_mask |= MXC_UARTURXD_BRK;
	}

	/*
	 * Characters to ignore
	 */
	umxc->port.ignore_status_mask = 0;
	if (termios->c_iflag & IGNPAR) {
		umxc->port.ignore_status_mask |= MXC_UARTURXD_FRMERR |
		    MXC_UARTURXD_PRERR;
	}
	if (termios->c_iflag & IGNBRK) {
		umxc->port.ignore_status_mask |= MXC_UARTURXD_BRK;
		/*
		 * If we are ignoring parity and break indicators,
		 * ignore overruns too (for real raw support)
		 */
		if (termios->c_iflag & IGNPAR) {
			umxc->port.ignore_status_mask |= MXC_UARTURXD_OVRRUN;
		}
	}

	/*
	 * Ignore all characters if CREAD is not set, still receive characters
	 * from the port, but throw them away.
	 */
	if ((termios->c_cflag & CREAD) == 0) {
		umxc->port.ignore_status_mask |= UART_CREAD_BIT;
	}

	/* Hardware flow control should controled by userspace */
	umxc->hardware_flow = (termios->c_cflag & CRTSCTS) ? 1 : 0;

	cr4 = readl(umxc->port.membase + MXC_UARTUCR4);
	if (UART_ENABLE_MS(port, termios->c_cflag)) {
		mxcuart_enable_ms(port);
		if (umxc->hardware_flow == 1) {
			cr4 = (cr4 & (~MXC_UARTUCR4_CTSTL_MASK)) |
			    (umxc->cts_threshold << MXC_UARTUCR4_CTSTL_OFFSET);
			cr2 |= MXC_UARTUCR2_CTSC;
			umxc->port.state->port.tty->hw_stopped = 0;
		} else {
			cr2 |= MXC_UARTUCR2_IRTS;
		}
	} else {
		cr2 |= MXC_UARTUCR2_IRTS;
	}

	/* Add Parity, character length and stop bits information */
	cr2 |= (readl(umxc->port.membase + MXC_UARTUCR2) & cr2_mask);
	writel(cr2, umxc->port.membase + MXC_UARTUCR2);
	/*
	   if (umxc->ir_mode == IRDA) {
	   ret = mxcuart_setir_special(baud);
	   if (ret == 0) {
	   cr4 &= ~MXC_UARTUCR4_IRSC;
	   } else {
	   cr4 |= MXC_UARTUCR4_IRSC;
	   }
	   } */
	writel(cr4, umxc->port.membase + MXC_UARTUCR4);

	/*
	 * Set baud rate
	 */

	/* Use integer scaling, if possible. Limit the denom to 15 bits. */
	num = 0;
	denom = (umxc->port.uartclk + 8 * baud) / (16 * baud) - 1;

	/* Use fractional scaling if needed to limit the max error to 0.5% */
	if (denom < 100) {
		u64 n64 = (u64) 16 * 0x8000 * baud + (umxc->port.uartclk / 2);
		do_div(n64, umxc->port.uartclk);
		num = (u_int) n64 - 1;
		denom = 0x7fff;
	}
	writel(num, umxc->port.membase + MXC_UARTUBIR);
	writel(denom, umxc->port.membase + MXC_UARTUBMR);

	spin_unlock_irqrestore(&umxc->port.lock, flags);
}

/*!
 * This function is called by the core driver to know the UART type.
 *
 * @param   port   the port structure for the UART passed in by the core driver
 *
 * @return  The function returns a pointer to a string describing the UART port.
 */
static const char *mxcuart_type(struct uart_port *port)
{
	return port->type == PORT_IMX ? "Freescale i.MX" : NULL;
}

/*!
 * This function is called by the core driver to release the memory resources
 * currently in use by the UART port.
 *
 * @param   port   the port structure for the UART passed in by the core driver
 */
static void mxcuart_release_port(struct uart_port *port)
{
	release_mem_region(port->mapbase, SZ_4K);
}

/*!
 * This function is called by the core driver to request memory resources for
 * the UART port.
 *
 * @param   port   the port structure for the UART passed in by the core driver
 *
 * @return  The function returns \b -EBUSY on failure, else it returns 0.
 */
static int mxcuart_request_port(struct uart_port *port)
{
	struct platform_device *pdev = to_platform_device(port->dev);
	struct resource *mmres;
	void *ret;

	mmres = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!mmres)
		return -ENODEV;

	ret = request_mem_region(mmres->start, mmres->end - mmres->start + 1,
							"serial_mxc");

	return  ret ? 0 : -EBUSY;
}

/*!
 * This function is called by the core driver to perform any autoconfiguration
 * steps required for the UART port. This function sets the port->type field.
 *
 * @param   port   the port structure for the UART passed in by the core driver
 * @param   flags  bit mask of the required configuration
 */
static void mxcuart_config_port(struct uart_port *port, int flags)
{
	if ((flags & UART_CONFIG_TYPE) && (mxcuart_request_port(port) == 0)) {
		port->type = PORT_IMX;
	}
}

/*!
 * This function is called by the core driver to verify that the new serial
 * port information contained within \a ser is suitable for this UART port type.
 * The function checks to see if the UART port type specified by the user
 * application while setting the UART port information matches what is stored
 * in the define \b PORT_MXC found in the header file include/linux/serial_core.h
 *
 * @param   port   the port structure for the UART passed in by the core driver
 * @param   ser    the new serial port information
 *
 * @return  The function returns 0 on success or \b -EINVAL if the port type
 *          specified is not equal to \b PORT_MXC.
 */
static int mxcuart_verify_port(struct uart_port *port,
			       struct serial_struct *ser)
{
	int ret = 0;
	if (ser->type != PORT_UNKNOWN && ser->type != PORT_IMX) {
		ret = -EINVAL;
	}
	return ret;
}

/*!
 * This function is used to send a high priority XON/XOFF character
 *
 * @param   port   the port structure for the UART passed in by the core driver
 * @param   ch     the character to send
 */
static void mxcuart_send_xchar(struct uart_port *port, char ch)
{
	unsigned long flags;

	port->x_char = ch;
	if (port->state->port.tty->hw_stopped) {
		return;
	}

	if (ch) {
		spin_lock_irqsave(&port->lock, flags);
		port->ops->start_tx(port);
		spin_unlock_irqrestore(&port->lock, flags);
	}
}

/*!
 * This function is used enable/disable the MXC UART clocks
 *
 * @param   port      the port structure for the UART passed in by the core driver
 * @param   state     New PM state
 * @param   oldstate  Current PM state
 */
static void
mxcuart_pm(struct uart_port *port, unsigned int state, unsigned int oldstate)
{
	uart_mxc_port *umxc = (uart_mxc_port *) port;

	if (state)
		clk_disable(umxc->clk);
	else
		clk_enable(umxc->clk);
}

/*!
 * This structure contains the pointers to the control functions that are
 * invoked by the core serial driver to access the UART hardware. The
 * structure is passed to serial_core.c file during registration.
 */
static struct uart_ops mxc_ops = {
	.tx_empty = mxcuart_tx_empty,
	.set_mctrl = mxcuart_set_mctrl,
	.get_mctrl = mxcuart_get_mctrl,
	.stop_tx = mxcuart_stop_tx,
	.start_tx = mxcuart_start_tx,
	.stop_rx = mxcuart_stop_rx,
	.enable_ms = mxcuart_enable_ms,
	.break_ctl = mxcuart_break_ctl,
	.startup = mxcuart_startup,
	.shutdown = mxcuart_shutdown,
	.set_termios = mxcuart_set_termios,
	.type = mxcuart_type,
	.pm = mxcuart_pm,
	.release_port = mxcuart_release_port,
	.request_port = mxcuart_request_port,
	.config_port = mxcuart_config_port,
	.verify_port = mxcuart_verify_port,
	.send_xchar = mxcuart_send_xchar,
};

#ifdef CONFIG_SERIAL_MXC_CONSOLE

/*
 * Write out a character once the UART is ready
 */
static inline void mxcuart_console_write_char(struct uart_port *port, char ch)
{
	volatile unsigned int status;

	do {
		status = readl(port->membase + MXC_UARTUSR1);
	} while ((status & MXC_UARTUSR1_TRDY) == 0);
	writel(ch, port->membase + MXC_UARTUTXD);
}

/*!
 * This function is called to write the console messages through the UART port.
 *
 * @param   co    the console structure
 * @param   s     the log message to be written to the UART
 * @param   count length of the message
 */
static void mxcuart_console_write(struct console *co, const char *s,
				  u_int count)
{
	struct uart_port *port = &mxc_ports[co->index]->port;
	volatile unsigned int status, oldcr1, oldcr2, oldcr3, cr2, cr3;
	int i;

	/*
	 * First save the control registers and then disable the interrupts
	 */
	oldcr1 = readl(port->membase + MXC_UARTUCR1);
	oldcr2 = readl(port->membase + MXC_UARTUCR2);
	oldcr3 = readl(port->membase + MXC_UARTUCR3);
	cr2 =
	    oldcr2 & ~(MXC_UARTUCR2_ATEN | MXC_UARTUCR2_RTSEN |
		       MXC_UARTUCR2_ESCI);
	cr3 =
	    oldcr3 & ~(MXC_UARTUCR3_DCD | MXC_UARTUCR3_RI |
		       MXC_UARTUCR3_DTRDEN);
	writel(MXC_UARTUCR1_UARTEN, port->membase + MXC_UARTUCR1);
	writel(cr2, port->membase + MXC_UARTUCR2);
	writel(cr3, port->membase + MXC_UARTUCR3);
	/*
	 * Do each character
	 */
	for (i = 0; i < count; i++) {
		mxcuart_console_write_char(port, s[i]);
		if (s[i] == '\n') {
			mxcuart_console_write_char(port, '\r');
		}
	}
	/*
	 * Finally, wait for the transmitter to become empty
	 */
	do {
		status = readl(port->membase + MXC_UARTUSR2);
	} while (!(status & MXC_UARTUSR2_TXDC));

	/*
	 * Restore the control registers
	 */
	writel(oldcr1, port->membase + MXC_UARTUCR1);
	writel(oldcr2, port->membase + MXC_UARTUCR2);
	writel(oldcr3, port->membase + MXC_UARTUCR3);
}

/*!
 * Initializes the UART port to be used to print console message with the
 * options specified. If no options are specified, then the function
 * initializes the UART with the default options of baudrate=115200, 8 bit
 * word size, no parity, no flow control.
 *
 * @param   co      The console structure
 * @param   options Any console options passed in from the command line
 *
 * @return  The function returns 0 on success or error.
 */
static int __init mxcuart_console_setup(struct console *co, char *options)
{
	uart_mxc_port *umxc;
	int baud = 115200;
	int bits = 8;
	int parity = 'n';
	int flow = 'n';
	volatile unsigned int cr = 0;

	/*
	 * Check whether an invalid uart number had been specified, and if
	 * so, search for the first available port that does have console
	 * support
	 */
	if (co->index >= MXC_UART_NR) {
		co->index = 0;
	}
	umxc = mxc_ports[co->index];

	if (umxc == NULL) {
		return -ENODEV;
	}

	clk_enable(umxc->clk);

	/* initialize port.lock else oops */
	spin_lock_init(&umxc->port.lock);

	/*
	 * Initialize the UART registers
	 */
	writel(MXC_UARTUCR1_UARTEN, umxc->port.membase + MXC_UARTUCR1);
	/* Enable the transmitter and do a software reset */
	writel(MXC_UARTUCR2_TXEN, umxc->port.membase + MXC_UARTUCR2);
	/* Wait till we are out of software reset */
	do {
		cr = readl(umxc->port.membase + MXC_UARTUCR2);
	} while (!(cr & MXC_UARTUCR2_SRST));

	writel(0x0, umxc->port.membase + MXC_UARTUCR3);
	writel(0x0, umxc->port.membase + MXC_UARTUCR4);
	/* Set TXTL to 2, RXTL to 1 and RFDIV to 2 */
	cr = 0x0800 | MXC_UARTUFCR_RFDIV | 0x1;
	if (umxc->mode == MODE_DTE) {
		cr |= MXC_UARTUFCR_DCEDTE;
	}
	writel(cr, umxc->port.membase + MXC_UARTUFCR);
	writel(0xFFFF, umxc->port.membase + MXC_UARTUSR1);
	writel(0xFFFF, umxc->port.membase + MXC_UARTUSR2);

	if (options != NULL) {
		uart_parse_options(options, &baud, &parity, &bits, &flow);
	}
	gpio_uart_active(umxc->port.line, umxc->ir_mode);
	return uart_set_options(&umxc->port, co, baud, parity, bits, flow);
}

static struct uart_driver mxc_reg;

/*!
 * This structure contains the pointers to the UART console functions. It is
 * passed as an argument when registering the console.
 */
static struct console mxc_console = {
	.name = "ttymxc",
	.write = mxcuart_console_write,
	.device = uart_console_device,
	.setup = mxcuart_console_setup,
	.flags = CON_PRINTBUFFER,
	.index = -1,
	.data = &mxc_reg,
};

/*!
 * This function registers the console callback functions with the kernel.
 */
static int __init mxcuart_console_init(void)
{
	register_console(&mxc_console);
	return 0;
}

console_initcall(mxcuart_console_init);

static int __init find_port(struct uart_port *p)
{
	int line;
	struct uart_port *port;
	for (line = 0; line < MXC_UART_NR; line++) {
		if (!mxc_ports[line])
			continue;
		port = &mxc_ports[line]->port;
		if (uart_match_port(p, port))
			return line;
	}
	return -ENODEV;
}

int __init mxc_uart_start_console(struct uart_port *port, char *options)
{
	int line;
	line = find_port(port);
	if (line < 0)
		return -ENODEV;

	add_preferred_console("ttymxc", line, options);
	printk("Switching Console to ttymxc%d at %s 0x%lx (options '%s')\n",
	       line, port->iotype == UPIO_MEM ? "MMIO" : "I/O port",
	       port->iotype ==
	       UPIO_MEM ? (unsigned long)port->mapbase : (unsigned long)port->
	       iobase, options);

	if (!(mxc_console.flags & CON_ENABLED)) {
		mxc_console.flags &= ~CON_PRINTBUFFER;
		register_console(&mxc_console);
	}
	return 0;
}

#define MXC_CONSOLE     (&mxc_console)
#else
#define MXC_CONSOLE     NULL
#endif				/* CONFIG_SERIAL_MXC_CONSOLE */

/*!
 * This structure contains the information such as the name of the UART driver
 * that appears in the /dev folder, major and minor numbers etc. This structure
 * is passed to the serial_core.c file.
 */
static struct uart_driver mxc_reg = {
	.owner = THIS_MODULE,
	.driver_name = "ttymxc",
	.dev_name = "ttymxc",
	.major = SERIAL_MXC_MAJOR,
	.minor = SERIAL_MXC_MINOR,
	.nr = MXC_UART_NR,
	.cons = MXC_CONSOLE,
};

/*!
 * This function is called to put the UART in a low power state. Refer to the
 * document driver-model/driver.txt in the kernel source tree for more
 * information.
 *
 * @param   pdev  the device structure used to give information on which UART
 *                to suspend
 * @param   state the power state the device is entering
 *
 * @return  The function returns 0 on success and -1 on failure
 */
static int mxcuart_suspend(struct platform_device *pdev, pm_message_t state)
{
	uart_mxc_port *umxc = platform_get_drvdata(pdev);

	if (umxc == NULL)
		return 0;	/* skip disabled ports */

	if (umxc && umxc->port.flags & ASYNC_INITIALIZED)
		uart_suspend_port(&mxc_reg, &umxc->port);

	if (umxc && umxc->port.flags & ASYNC_SUSPENDED)
		umxc->port.state->port.tty->hw_stopped = 1;

	return 0;
}

/*!
 * This function is called to bring the UART back from a low power state. Refer
 * to the document driver-model/driver.txt in the kernel source tree for more
 * information.
 *
 * @param   pdev  the device structure used to give information on which UART
 *                to resume
 *
 * @return  The function returns 0 on success and -1 on failure
 */
static int mxcuart_resume(struct platform_device *pdev)
{
	uart_mxc_port *umxc = platform_get_drvdata(pdev);

	if (umxc == NULL)
		return 0;	/* skip disabled ports */

	if (umxc && umxc->port.flags & ASYNC_SUSPENDED) {
		umxc->port.state->port.tty->hw_stopped = 0;
		uart_resume_port(&mxc_reg, &umxc->port);
	}

	return 0;
}

/*!
 * This function is called during the driver binding process. Based on the UART
 * that is being probed this function adds the appropriate UART port structure
 * in the core driver.
 *
 * @param   pdev  the device structure used to store device specific
 *                information that is used by the suspend, resume and remove
 *                functions
 *
 * @return  The function returns 0 if successful; -1 otherwise.
 */
static int mxcuart_probe(struct platform_device *pdev)
{
	int id = pdev->id;
	struct resource *res;
	void __iomem *base;

	mxc_ports[id] = pdev->dev.platform_data;
	mxc_ports[id]->port.ops = &mxc_ops;

	/* Do not use UARTs that are disabled during integration */
	if (mxc_ports[id]->enabled == 1) {
		res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
		if (!res)
			return -ENODEV;

		base = ioremap(res->start, res->end - res->start + 1);
		if (!base)
			return -ENOMEM;

		mxc_ports[id]->port.membase = base;
		mxc_ports[id]->port.mapbase = res->start;
		mxc_ports[id]->port.dev = &pdev->dev;
		mxc_ports[id]->port.irq = platform_get_irq(pdev, 0);
		mxc_ports[id]->irqs[0] = platform_get_irq(pdev, 1);
		mxc_ports[id]->irqs[1] = platform_get_irq(pdev, 2);
		spin_lock_init(&mxc_ports[id]->port.lock);
		/* Enable the low latency flag for DMA UART ports */
		if (mxc_ports[id]->dma_enabled == 1) {
			mxc_ports[id]->port.flags |= ASYNC_LOW_LATENCY;
		}

		mxc_ports[id]->clk = clk_get(&pdev->dev, NULL);
		if (mxc_ports[id]->clk == NULL)
			return -1;

		uart_add_one_port(&mxc_reg, &mxc_ports[id]->port);
		platform_set_drvdata(pdev, mxc_ports[id]);
	}
	return 0;
}

/*!
 * Dissociates the driver from the UART device. Removes the appropriate UART
 * port structure from the core driver.
 *
 * @param   pdev  the device structure used to give information on which UART
 *                to remove
 *
 * @return  The function always returns 0.
 */
static int mxcuart_remove(struct platform_device *pdev)
{
	uart_mxc_port *umxc = platform_get_drvdata(pdev);

	platform_set_drvdata(pdev, NULL);

	if (umxc) {
		uart_remove_one_port(&mxc_reg, &umxc->port);
		iounmap(umxc->port.membase);
	}
	return 0;
}

/*!
 * This structure contains pointers to the power management callback functions.
 */
static struct platform_driver mxcuart_driver = {
	.driver = {
		   .name = "mxcintuart",
		   },
	.probe = mxcuart_probe,
	.remove = mxcuart_remove,
	.suspend = mxcuart_suspend,
	.resume = mxcuart_resume,
};

/*!
 * This function is used to initialize the UART driver module. The function
 * registers the power management callback functions with the kernel and also
 * registers the UART callback functions with the core serial driver.
 *
 * @return  The function returns 0 on success and a non-zero value on failure.
 */
static int __init mxcuart_init(void)
{
	int ret = 0;

	printk(KERN_INFO "Serial: MXC Internal UART driver\n");
	ret = uart_register_driver(&mxc_reg);
	if (ret == 0) {
		/* Register the device driver structure. */
		ret = platform_driver_register(&mxcuart_driver);
		if (ret != 0) {
			uart_unregister_driver(&mxc_reg);
		}
	}
	return ret;
}

/*!
 * This function is used to cleanup all resources before the driver exits.
 */
static void __exit mxcuart_exit(void)
{
	platform_driver_unregister(&mxcuart_driver);
	uart_unregister_driver(&mxc_reg);
}

module_init(mxcuart_init);
module_exit(mxcuart_exit);

MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("MXC serial port driver");
MODULE_LICENSE("GPL");
