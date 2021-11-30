// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright 2020 NXP
 */

#include <linux/module.h>
#include <linux/ioport.h>
#include <linux/init.h>
#include <linux/serial_core.h>
#include <linux/serial.h>
#include <linux/delay.h>
#include <linux/of.h>
#include <linux/io.h>

#define URTX0 0x40 /* Transmitter Register */
#define UTS_TXFULL (1<<4) /* TxFIFO full */
#define IMX21_UTS 0xb4 /* UART Test Register on all other i.mx*/

/* IMX lpuart has four extra unused regs located at the beginning */
#define IMX_REG_OFF	0x10
#define UARTSTAT		0x04
#define UARTDATA		0x0C
#define UARTSTAT_TDRE		0x00800000

static void imx_uart_console_early_putchar(struct uart_port *port, unsigned char ch)
{
	while (readl_relaxed(port->membase + IMX21_UTS) & UTS_TXFULL)
		cpu_relax();

	writel_relaxed(ch, port->membase + URTX0);
}

static void imx_uart_console_early_write(struct console *con, const char *s,
					 unsigned count)
{
	struct earlycon_device *dev = con->data;

	uart_console_write(&dev->port, s, count, imx_uart_console_early_putchar);
}

static int __init
imx_console_early_setup(struct earlycon_device *dev, const char *opt)
{
	if (!dev->port.membase)
		return -ENODEV;

	dev->con->write = imx_uart_console_early_write;

	return 0;
}

static inline u32 lpuart32_read(struct uart_port *port, u32 off)
{
	switch (port->iotype) {
	case UPIO_MEM32:
		return readl(port->membase + off);
	case UPIO_MEM32BE:
		return ioread32be(port->membase + off);
	default:
		return 0;
	}
}

static void lpuart32_wait_bit_set(struct uart_port *port, unsigned int offset,
				  u32 bit)
{
	while (!(lpuart32_read(port, offset) & bit))
		cpu_relax();
}

static inline void lpuart32_write(struct uart_port *port, u32 val,
				  u32 off)
{
	switch (port->iotype) {
	case UPIO_MEM32:
		writel(val, port->membase + off);
		break;
	case UPIO_MEM32BE:
		iowrite32be(val, port->membase + off);
		break;
	}
}

static void lpuart32_console_putchar(struct uart_port *port, unsigned char ch)
{
	lpuart32_wait_bit_set(port, UARTSTAT, UARTSTAT_TDRE);
	lpuart32_write(port, ch, UARTDATA);
}

static void lpuart32_early_write(struct console *con, const char *s, unsigned n)
{
	struct earlycon_device *dev = con->data;

	uart_console_write(&dev->port, s, n, lpuart32_console_putchar);
}

static int __init lpuart32_imx_early_console_setup(struct earlycon_device *device,
						   const char *opt)
{
	if (!device->port.membase)
		return -ENODEV;

	device->port.iotype = UPIO_MEM32;
	device->port.membase += IMX_REG_OFF;
	device->con->write = lpuart32_early_write;

	return 0;
}

OF_EARLYCON_DECLARE(ec_imx6q, "fsl,imx6q-uart", imx_console_early_setup);
OF_EARLYCON_DECLARE(ec_imx21, "fsl,imx21-uart", imx_console_early_setup);
OF_EARLYCON_DECLARE(lpuart32, "fsl,imx7ulp-lpuart", lpuart32_imx_early_console_setup);
OF_EARLYCON_DECLARE(lpuart32, "fsl,imx8ulp-lpuart", lpuart32_imx_early_console_setup);
OF_EARLYCON_DECLARE(lpuart32, "fsl,imx8qxp-lpuart", lpuart32_imx_early_console_setup);
OF_EARLYCON_DECLARE(lpuart32, "fsl,imxrt1050-lpuart", lpuart32_imx_early_console_setup);

MODULE_AUTHOR("NXP");
MODULE_DESCRIPTION("IMX earlycon driver");
MODULE_LICENSE("GPL");
