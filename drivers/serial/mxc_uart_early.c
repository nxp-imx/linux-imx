/*
 * Copyright 2004-2009 Freescale Semiconductor, Inc. All Rights Reserved.
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
};

int __init mxc_uart_start_console(struct uart_port *, char *);
static struct mxc_early_uart_device mxc_early_device __initdata;
static int mxc_early_uart_registered __initdata;
static struct clk *clk;

/*
 * Write out a character once the UART is ready
 */
static void __init mxcuart_console_write_char(struct uart_port *port, int ch)
{
	unsigned int status;

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

static int __init parse_options(struct mxc_early_uart_device *device,
				char *options)
{
	struct uart_port *port = &device->port;
	int mapsize = 64;
	int length;

	if (!options)
		return -ENODEV;

	port->uartclk = 5600000;
	port->iotype = UPIO_MEM;
	port->mapbase = simple_strtoul(options, &options, 0);
	port->membase = ioremap(port->mapbase, mapsize);

	if ((options = strchr(options, ','))) {
		options++;
		device->baud = simple_strtoul(options, NULL, 0);
		length = min(strcspn(options, " "), sizeof(device->options));
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

static int __init mxc_early_uart_setup(struct console *console, char *options)
{
	struct mxc_early_uart_device *device = &mxc_early_device;
	int err;
	if (device->port.membase || device->port.iobase)
		return 0;
	if ((err = parse_options(device, options)) < 0)
		return err;
	return 0;
}

static struct console mxc_early_uart_console __initdata = {
	.name = "mxcuart",
	.write = early_mxcuart_console_write,
	.setup = mxc_early_uart_setup,
	.flags = CON_PRINTBUFFER,
	.index = -1,
};

static int __init mxc_early_uart_console_init(void)
{

	if (!mxc_early_uart_registered) {
		register_console(&mxc_early_uart_console);
		mxc_early_uart_registered = 1;
	}

	return 0;
}

int __init mxc_early_serial_console_init(char *cmdline)
{
	char *options;
	int err;
	int uart_paddr;

	options = strstr(cmdline, "console=mxcuart");
	if (!options)
		return -ENODEV;

	/* Extracting MXC UART Uart Port Address from cmdline */
	options = strchr(cmdline, ',') + 1;
	uart_paddr = simple_strtoul(options, NULL, 16);

#ifdef UART1_BASE_ADDR
	if (uart_paddr == UART1_BASE_ADDR)
		clk = clk_get(NULL, "uart_clk.0");
#endif
#ifdef UART2_BASE_ADDR
	if (uart_paddr == UART2_BASE_ADDR)
		clk = clk_get(NULL, "uart_clk.1");
#endif
#ifdef UART3_BASE_ADDR
	if (uart_paddr == UART3_BASE_ADDR)
		clk = clk_get(NULL, "uart_clk.2");
#endif
	if (clk == NULL)
		return -1;

	/* Enable Early MXC UART Clock */
	clk_enable(clk);

	options = strchr(cmdline, ',') + 1;
	if ((err = mxc_early_uart_setup(NULL, options)) < 0)
		return err;
	return mxc_early_uart_console_init();
}

int __init mxc_early_uart_console_switch(void)
{
	struct mxc_early_uart_device *device = &mxc_early_device;
	struct uart_port *port = &device->port;
	int mmio, line;

	if (!(mxc_early_uart_console.flags & CON_ENABLED))
		return 0;
	/* Try to start the normal driver on a matching line.  */
	mmio = (port->iotype == UPIO_MEM);
	line = mxc_uart_start_console(port, device->options);

	if (line < 0)
		printk("No ttymxc device at %s 0x%lx for console\n",
		       mmio ? "MMIO" : "I/O port",
		       mmio ? port->mapbase : (unsigned long)port->iobase);

	unregister_console(&mxc_early_uart_console);
	if (mmio)
		iounmap(port->membase);

	clk_disable(clk);
	clk_put(clk);

	return 0;
}

late_initcall(mxc_early_uart_console_switch);
