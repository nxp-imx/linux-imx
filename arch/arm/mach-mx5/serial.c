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
 * @file mach-mx51/serial.c
 *
 * @brief This file contains the UART initiliazation.
 *
 * @ingroup MSL_MX51
 */
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/serial.h>
#include <mach/hardware.h>
#include <mach/mxc_uart.h>
#include "serial.h"

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
	       .ints_muxed = 1,
	       .mode = MODE_DCE,
	       .ir_mode = NO_IRDA,
	       .enabled = 1,
	       .cts_threshold = UART1_UCR4_CTSTL,
	       .dma_enabled = UART1_DMA_ENABLE,
	       .dma_rxbuf_size = UART1_DMA_RXBUFSIZE,
	       .rx_threshold = UART1_UFCR_RXTL,
	       .tx_threshold = UART1_UFCR_TXTL,
	       .dma_tx_id = MXC_DMA_UART1_TX,
	       .dma_rx_id = MXC_DMA_UART1_RX,
	       .rxd_mux = MXC_UART_RXDMUX,
	       },
	[1] = {
	       .port = {
			.iotype = SERIAL_IO_MEM,
			.fifosize = 32,
			.flags = ASYNC_BOOT_AUTOCONF,
			.line = 1,
			},
	       .ints_muxed = 1,
	       .mode = MODE_DCE,
	       .ir_mode = NO_IRDA,
	       .enabled = 1,
	       .cts_threshold = UART2_UCR4_CTSTL,
	       .dma_enabled = UART2_DMA_ENABLE,
	       .dma_rxbuf_size = UART2_DMA_RXBUFSIZE,
	       .rx_threshold = UART2_UFCR_RXTL,
	       .tx_threshold = UART2_UFCR_TXTL,
	       .dma_tx_id = MXC_DMA_UART2_TX,
	       .dma_rx_id = MXC_DMA_UART2_RX,
	       .rxd_mux = MXC_UART_RXDMUX,
	       },
	[2] = {
	       .port = {
			.iotype = SERIAL_IO_MEM,
			.fifosize = 32,
			.flags = ASYNC_BOOT_AUTOCONF,
			.line = 2,
			},
	       .ints_muxed = 1,
	       .mode = MODE_DCE,
	       .ir_mode = NO_IRDA,
	       .enabled = 1,
	       .cts_threshold = UART3_UCR4_CTSTL,
	       .dma_enabled = UART3_DMA_ENABLE,
	       .dma_rxbuf_size = UART3_DMA_RXBUFSIZE,
	       .rx_threshold = UART3_UFCR_RXTL,
	       .tx_threshold = UART3_UFCR_TXTL,
	       .dma_tx_id = MXC_DMA_UART3_TX,
	       .dma_rx_id = MXC_DMA_UART3_RX,
	       .rxd_mux = MXC_UART_RXDMUX,
	       },
	[3] = {
	       .port = {
			.iotype = SERIAL_IO_MEM,
			.fifosize = 32,
			.flags = ASYNC_BOOT_AUTOCONF,
			.line = 3,
			},
	       .ints_muxed = 1,
	       .mode = MODE_DCE,
	       .ir_mode = NO_IRDA,
	       .enabled = 1,
	       .cts_threshold = UART4_UCR4_CTSTL,
	       .dma_enabled = UART4_DMA_ENABLE,
	       .dma_rxbuf_size = UART4_DMA_RXBUFSIZE,
	       .rx_threshold = UART4_UFCR_RXTL,
	       .tx_threshold = UART4_UFCR_TXTL,
	       .dma_tx_id = MXC_DMA_UART4_TX,
	       .dma_rx_id = MXC_DMA_UART4_RX,
	       .rxd_mux = MXC_UART_RXDMUX,
	       },
	[4] = {
	       .port = {
			.iotype = SERIAL_IO_MEM,
			.fifosize = 32,
			.flags = ASYNC_BOOT_AUTOCONF,
			.line = 4,
			},
	       .ints_muxed = 1,
	       .mode = MODE_DCE,
	       .ir_mode = NO_IRDA,
	       .enabled = 1,
	       .cts_threshold = UART5_UCR4_CTSTL,
	       .dma_enabled = UART5_DMA_ENABLE,
	       .dma_rxbuf_size = UART5_DMA_RXBUFSIZE,
	       .rx_threshold = UART5_UFCR_RXTL,
	       .tx_threshold = UART5_UFCR_TXTL,
	       .dma_tx_id = MXC_DMA_UART5_TX,
	       .dma_rx_id = MXC_DMA_UART5_RX,
	       .rxd_mux = MXC_UART_RXDMUX,
	       },
};

static struct resource mxc_uart_resources1[] = {
	{
		.start = UART1_BASE_ADDR,
		.end = UART1_BASE_ADDR + 0x0B5,
		.flags = IORESOURCE_MEM,
	},
	{
		.start = MXC_INT_UART1,
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
		.start = MXC_INT_UART2,
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

static struct resource mxc_uart_resources3[] = {
	{
		.start = UART3_BASE_ADDR,
		.end = UART3_BASE_ADDR + 0x0B5,
		.flags = IORESOURCE_MEM,
	},
	{
		.start = MXC_INT_UART3,
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

static struct resource mxc_uart_resources4[] = {
	{
		.start = UART4_BASE_ADDR,
		.end = UART4_BASE_ADDR + 0x0B5,
		.flags = IORESOURCE_MEM,
	},
	{
		.start = MXC_INT_UART4,
		.flags = IORESOURCE_IRQ,
	},
};

static struct platform_device mxc_uart_device4 = {
	.name = "mxcintuart",
	.id = 3,
       .num_resources = ARRAY_SIZE(mxc_uart_resources4),
	.resource = mxc_uart_resources4,
	.dev = {
		.platform_data = &mxc_ports[3],
		},
};

static struct resource mxc_uart_resources5[] = {
	{
		.start = UART5_BASE_ADDR,
		.end = UART5_BASE_ADDR + 0x0B5,
		.flags = IORESOURCE_MEM,
	},
	{
		.start = MXC_INT_UART5,
		.flags = IORESOURCE_IRQ,
	},
};

static struct platform_device mxc_uart_device5 = {
	.name = "mxcintuart",
	.id = 4,
       .num_resources = ARRAY_SIZE(mxc_uart_resources5),
	.resource = mxc_uart_resources5,
	.dev = {
		.platform_data = &mxc_ports[4],
		},
};

static int __init mxc_init_uart(void)
{
	if (cpu_is_mx53() || cpu_is_mx50()) {
		mxc_uart_resources1[0].start -= 0x20000000;
		mxc_uart_resources1[0].end -= 0x20000000;
		mxc_uart_resources2[0].start -= 0x20000000;
		mxc_uart_resources2[0].end -= 0x20000000;
		mxc_uart_resources3[0].start -= 0x20000000;
		mxc_uart_resources3[0].end -= 0x20000000;
		mxc_uart_resources4[0].start -= 0x20000000;
		mxc_uart_resources4[0].end -= 0x20000000;
		mxc_uart_resources5[0].start -= 0x20000000;
		mxc_uart_resources5[0].end -= 0x20000000;
	}

	/* Register all the MXC UART platform device structures */
	platform_device_register(&mxc_uart_device1);
	platform_device_register(&mxc_uart_device2);
	platform_device_register(&mxc_uart_device3);
	if (cpu_is_mx53()) {
		platform_device_register(&mxc_uart_device4);
		platform_device_register(&mxc_uart_device5);
	}
	return 0;
}

#else
static int __init mxc_init_uart(void)
{
	return 0;
}
#endif

arch_initcall(mxc_init_uart);
