/*
 * Copyright (C) 2004-2010 Freescale Semiconductor, Inc. All Rights Reserved.
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
 * @file drivers/serial/mxc_uart_early.c
 *
 * @brief Driver for the Freescale Semiconductor MXC serial ports based on
 * drivers/char/8250_early.c, Copyright 2004 Hewlett-Packard Development Company,
 * L.P.	by Bjorn Helgaasby.
 *
 * Early serial console for MXC UARTS.
 *
 * This is for use before the serial driver has initialized, in
 * particular, before the UARTs have been discovered and named.
 * Instead of specifying the console device as, e.g., "ttymxc0",
 * we locate the device directly by its MMIO or I/O port address.
 *
 * The user can specify the device directly, e.g.,
 *	console=mxcuart,0x43f90000,115200n8
 * or platform code can call early_uart_console_init() to set
 * the early UART device.
 *
 * After the normal serial driver starts, we try to locate the
 * matching ttymxc device and start a console there.
 */

/*
 * Include Files
 */

#include <linux/tty.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/console.h>
#include <linux/serial_core.h>
#include <linux/serial_reg.h>
#include <linux/clk.h>
#include <mach/mxc_uart.h>

struct mxc_early_uart_device {
	struct uart_port port;
	char options[16];	/* e.g., 115200n8 */
	unsigned int baud;
	struct clk *clk;
};
static struct mxc_early_uart_device mxc_early_device __initdata;

/*
 * Write out a character once the UART is ready
 */
static void __init mxcuart_console_write_char(struct uart_port *port, int ch)
{
	unsigned int status;

	do {
		status = readl(port->membase + MXC_UARTUSR2);
	} while ((status & MXC_UARTUSR2_TXFE) == 0);
	writel(ch, port->membase + MXC_UARTUTXD);
}

/*!
 * This function is called to write the console messages through the UART port.
 *
 * @param   co    the console structure
 * @param   s     the log message to be written to the UART
 * @param   count length of the message
 */
void __init early_mxcuart_console_write(struct console *co, const char *s,
					u_int count)
{
	struct uart_port *port = &mxc_early_device.port;
	volatile unsigned int status, oldcr1, oldcr2, oldcr3, cr2, cr3;

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

	/* Transmit string */
	uart_console_write(port, s, count, mxcuart_console_write_char);

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

static unsigned int __init probe_baud(struct uart_port *port)
{
	/* FIXME Return Default Baud Rate */
	return 115200;
}

static int __init mxc_early_uart_setup(struct console *console, char *options)
{
	struct mxc_early_uart_device *device = &mxc_early_device;
	struct uart_port *port = &device->port;
	int length;

	if (device->port.membase || device->port.iobase)
		return -ENODEV;

	/* Enable Early MXC UART Clock */
	clk_enable(device->clk);

	port->uartclk = 5600000;
	port->iotype = UPIO_MEM;
	port->membase = ioremap(port->mapbase, SZ_4K);

	if (options) {
		device->baud = simple_strtoul(options, NULL, 0);
		length = min(strlen(options), sizeof(device->options));
		strncpy(device->options, options, length);
	} else {
		device->baud = probe_baud(port);
		snprintf(device->options, sizeof(device->options), "%u",
			 device->baud);
	}
	printk(KERN_INFO
	       "MXC_Early serial console at MMIO 0x%x (options '%s')\n",
	       port->mapbase, device->options);
	return 0;
}

static struct console mxc_early_uart_console __initdata = {
	.name = "ttymxc",
	.write = early_mxcuart_console_write,
	.setup = mxc_early_uart_setup,
	.flags = CON_PRINTBUFFER | CON_BOOT,
	.index = -1,
};

int __init mxc_early_serial_console_init(unsigned long base, struct clk *clk)
{
	mxc_early_device.clk = clk;
	mxc_early_device.port.mapbase = base;

	register_console(&mxc_early_uart_console);
	return 0;
}

int __init mxc_early_uart_console_disable(void)
{
	struct mxc_early_uart_device *device = &mxc_early_device;
	struct uart_port *port = &device->port;

	if (mxc_early_uart_console.index >= 0) {
		iounmap(port->membase);
		clk_disable(device->clk);
		clk_put(device->clk);
	}
	return 0;
}
late_initcall(mxc_early_uart_console_disable);
