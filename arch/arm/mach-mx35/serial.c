/*
 * Copyright (C) 2008-2010 Freescale Semiconductor, Inc. All Rights Reserved.
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
 * @file mach-mx35/serial.c
 *
 * @brief This file contains the UART initiliazation.
 *
 * @ingroup MSL_MX35
 */
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/serial.h>
#include <mach/hardware.h>
#include <mach/mxc_uart.h>
#include "serial.h"
#include "board-mx35_3stack.h"

#if defined(CONFIG_SERIAL_MXC) || defined(CONFIG_SERIAL_MXC_MODULE)

/*!
 * This is an array where each element holds information about a UART port,
 * like base address of the UART, interrupt numbers etc. This structure is
 * passed to the serial_core.c file. Based on which UART is used, the core file
 * passes back the appropriate port structure as an argument to the control
 * functions.
 */
static uart_mxc_port mxc_ports[] = {
	[0] = {
	       .port = {
			.iotype = SERIAL_IO_MEM,
			.fifosize = 32,
			.flags = ASYNC_BOOT_AUTOCONF,
			.line = 0,
			},
	       .ints_muxed = UART1_MUX_INTS,
	       .mode = UART1_MODE,
	       .ir_mode = UART1_IR,
	       .enabled = UART1_ENABLED,
	       .hardware_flow = UART1_HW_FLOW,
	       .cts_threshold = UART1_UCR4_CTSTL,
	       .dma_enabled = UART1_DMA_ENABLE,
	       .dma_rxbuf_size = UART1_DMA_RXBUFSIZE,
	       .rx_threshold = UART1_UFCR_RXTL,
	       .tx_threshold = UART1_UFCR_TXTL,
	       .dma_tx_id = MXC_DMA_UART1_TX,
	       .dma_rx_id = MXC_DMA_UART1_RX,
	       .rxd_mux = MXC_UART_RXDMUX,
	       .ir_tx_inv = MXC_IRDA_TX_INV,
	       .ir_rx_inv = MXC_IRDA_RX_INV,
	       },
	[1] = {
	       .port = {
			.iotype = SERIAL_IO_MEM,
			.fifosize = 32,
			.flags = ASYNC_BOOT_AUTOCONF,
			.line = 1,
			},
	       .ints_muxed = UART2_MUX_INTS,
	       .mode = UART2_MODE,
	       .ir_mode = UART2_IR,
	       .enabled = UART2_ENABLED,
	       .hardware_flow = UART2_HW_FLOW,
	       .cts_threshold = UART2_UCR4_CTSTL,
	       .dma_enabled = UART2_DMA_ENABLE,
	       .dma_rxbuf_size = UART2_DMA_RXBUFSIZE,
	       .rx_threshold = UART2_UFCR_RXTL,
	       .tx_threshold = UART2_UFCR_TXTL,
	       .dma_tx_id = MXC_DMA_UART2_TX,
	       .dma_rx_id = MXC_DMA_UART2_RX,
	       .rxd_mux = MXC_UART_IR_RXDMUX,
	       .ir_tx_inv = MXC_IRDA_TX_INV,
	       .ir_rx_inv = MXC_IRDA_RX_INV,
	       },
#if UART3_ENABLED == 1
	[2] = {
	       .port = {
			.iotype = SERIAL_IO_MEM,
			.fifosize = 32,
			.flags = ASYNC_BOOT_AUTOCONF,
			.line = 2,
			},
	       .ints_muxed = UART3_MUX_INTS,
	       .mode = UART3_MODE,
	       .ir_mode = UART3_IR,
	       .enabled = UART3_ENABLED,
	       .hardware_flow = UART3_HW_FLOW,
	       .cts_threshold = UART3_UCR4_CTSTL,
	       .dma_enabled = UART3_DMA_ENABLE,
	       .dma_rxbuf_size = UART3_DMA_RXBUFSIZE,
	       .rx_threshold = UART3_UFCR_RXTL,
	       .tx_threshold = UART3_UFCR_TXTL,
	       .dma_tx_id = MXC_DMA_UART3_TX,
	       .dma_rx_id = MXC_DMA_UART3_RX,
	       .rxd_mux = MXC_UART_RXDMUX,
	       .ir_tx_inv = MXC_IRDA_TX_INV,
	       .ir_rx_inv = MXC_IRDA_RX_INV,
	       },
#endif
};

static struct resource mxc_uart_resources1[] = {
	{
		.start = UART1_BASE_ADDR,
		.end = UART1_BASE_ADDR + 0x0B5,
		.flags = IORESOURCE_MEM,
	},
	{
		.start = UART1_INT1,
		.flags = IORESOURCE_IRQ,
	},
	{
		.start = UART1_INT2,
		.flags = IORESOURCE_IRQ,
	},
	{
		.start = UART1_INT3,
		.flags = IORESOURCE_IRQ,
	},

};

static struct platform_device mxc_uart_device1 = {
	.name = "mxcintuart",
	.id = 0,
	.num_resources = ARRAY_SIZE(mxc_uart_resources1),
	.resource = mxc_uart_resources1,
	.dev = {
		.platform_data = &mxc_ports[0],
		},
};

static struct resource mxc_uart_resources2[] = {
	{
		.start = UART2_BASE_ADDR,
		.end = UART2_BASE_ADDR + 0x0B5,
		.flags = IORESOURCE_MEM,
	},
	{
		.start = UART2_INT1,
		.flags = IORESOURCE_IRQ,
	},
	{
		.start = UART2_INT2,
		.flags = IORESOURCE_IRQ,
	},
	{
		.start = UART2_INT3,
		.flags = IORESOURCE_IRQ,
	},
};

static struct platform_device mxc_uart_device2 = {
	.name = "mxcintuart",
	.id = 1,
       .num_resources = ARRAY_SIZE(mxc_uart_resources2),
	.resource = mxc_uart_resources2,
	.dev = {
		.platform_data = &mxc_ports[1],
		},
};

#if UART3_ENABLED == 1
static struct resource mxc_uart_resources3[] = {
	{
		.start = UART3_BASE_ADDR,
		.end = UART3_BASE_ADDR + 0x0B5,
		.flags = IORESOURCE_MEM,
	},
	{
		.start = UART3_INT1,
		.flags = IORESOURCE_IRQ,
	},
	{
		.start = UART3_INT2,
		.flags = IORESOURCE_IRQ,
	},
	{
		.start = UART3_INT3,
		.flags = IORESOURCE_IRQ,
	},
};

static struct platform_device mxc_uart_device3 = {
	.name = "mxcintuart",
	.id = 2,
       .num_resources = ARRAY_SIZE(mxc_uart_resources3),
	.resource = mxc_uart_resources3,
	.dev = {
		.platform_data = &mxc_ports[2],
		},
};
#endif

static int __init mxc_init_uart(void)
{
	/* Register all the MXC UART platform device structures */
	platform_device_register(&mxc_uart_device1);
	platform_device_register(&mxc_uart_device2);

	/* Grab ownership of shared UARTs 3 and 4, only when enabled */
#if UART3_ENABLED == 1
	platform_device_register(&mxc_uart_device3);
#endif				/* UART3_ENABLED */

	return 0;
}

#else
static int __init mxc_init_uart(void)
{
	return 0;
}
#endif

arch_initcall(mxc_init_uart);
