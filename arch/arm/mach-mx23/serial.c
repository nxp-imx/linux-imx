/*
  * Copyright (C) 2010 Freescale Semiconductor, Inc. All Rights Reserved.
  *
  * This program is distributed in the hope that it will be useful,
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

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/irq.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/platform_device.h>

#include <mach/hardware.h>
#include <mach/device.h>
#include <mach/dma.h>
#include "device.h"

#if defined(CONFIG_SERIAL_MXS_AUART) || \
	defined(CONFIG_SERIAL_MXS_AUART_MODULE)

#ifdef CONFIG_MXS_AUART1_DEVICE_ENABLE
static struct resource auart1_resource[] = {
	{
	 .flags = IORESOURCE_MEM,
	 .start = AUART1_PHYS_ADDR,
	 .end	= AUART1_PHYS_ADDR + 0xFFF,
	 },
	{
	 .flags = IORESOURCE_DMA,
	 .start = MXS_DMA_CHANNEL_AHB_APBX_UART1_RX,
	 .end = MXS_DMA_CHANNEL_AHB_APBX_UART1_RX,
	 },
	{
	 .flags = IORESOURCE_DMA,
	 .start = MXS_DMA_CHANNEL_AHB_APBX_UART1_TX,
	 .end = MXS_DMA_CHANNEL_AHB_APBX_UART1_TX,
	 },
	{
	 .flags = IORESOURCE_IRQ,
	 .start = IRQ_UARTAPP_INTERNAL,
	 .end	= IRQ_UARTAPP_INTERNAL,
	 },
	{
	 .flags = IORESOURCE_IRQ,
	 .start = IRQ_UARTAPP_RX_DMA,
	 .end	= IRQ_UARTAPP_RX_DMA,
	 },
	{
	 .flags = IORESOURCE_IRQ,
	 .start = IRQ_UARTAPP_TX_DMA,
	 .end	= IRQ_UARTAPP_TX_DMA,
	 },
};

static struct mxs_auart_plat_data mxs_auart1_platdata = {
	.fifo_size = 16,
#ifdef CONFIG_MXS_AUART1_DMA_ENABLE
	.dma_mode = 1,
#endif
	.dma_rx_buffer_size = 8,
	.timeout = HZ,
};
#endif

#ifdef CONFIG_MXS_AUART2_DEVICE_ENABLE
static struct resource auart2_resource[] = {
	{
	 .flags = IORESOURCE_MEM,
	 .start = AUART2_PHYS_ADDR,
	 .end	= AUART2_PHYS_ADDR + 0xFFF,
	 },
	{
	 .flags = IORESOURCE_DMA,
	 .start = MXS_DMA_CHANNEL_AHB_APBX_UART2_RX,
	 .end = MXS_DMA_CHANNEL_AHB_APBX_UART2_RX,
	 },
	{
	 .flags = IORESOURCE_DMA,
	 .start = MXS_DMA_CHANNEL_AHB_APBX_UART2_TX,
	 .end = MXS_DMA_CHANNEL_AHB_APBX_UART2_TX,
	 },
	{
	 .flags = IORESOURCE_IRQ,
	 .start = IRQ_UARTAPP2_INTERNAL,
	 .end	= IRQ_UARTAPP2_INTERNAL,
	 },
	{
	 .flags = IORESOURCE_IRQ,
	 .start = IRQ_UARTAPP2_RX_DMA,
	 .end	= IRQ_UARTAPP2_RX_DMA,
	 },
	{
	 .flags = IORESOURCE_IRQ,
	 .start = IRQ_UARTAPP2_TX_DMA,
	 .end	= IRQ_UARTAPP2_TX_DMA,
	 },
};

static struct mxs_auart_plat_data mxs_auart2_platdata = {
	.fifo_size = 16,
#ifdef CONFIG_MXS_AUART2_DMA_ENABLE
	.dma_mode = 1,
#endif
	.dma_rx_buffer_size = 8,
	.timeout = HZ,
};
#endif

void __init mx23_init_auart(void)
{
	int i;
	struct mxs_dev_lookup *plookup;
	struct platform_device *pdev;

	plookup = mxs_get_devices("mxs-auart");
	if (plookup == NULL || IS_ERR(plookup))
		return;
	for (i = 0; i < plookup->size; i++) {
		pdev = plookup->pdev + i;
		switch (pdev->id) {
#ifdef CONFIG_MXS_AUART1_DEVICE_ENABLE
		case 1:
			pdev->resource = auart1_resource;
			pdev->num_resources = ARRAY_SIZE(auart1_resource);
			pdev->dev.platform_data = &mxs_auart1_platdata;
			break;
#endif
#ifdef CONFIG_MXS_AUART2_DEVICE_ENABLE
		case 2:
			pdev->resource = auart2_resource;
			pdev->num_resources = ARRAY_SIZE(auart2_resource);
			pdev->dev.platform_data = &mxs_auart2_platdata;
			break;
#endif
		default:
			break;
		}
		mxs_add_device(pdev, 3);
	}
}
#else
void __init mx23_init_auart(void)
{
}
#endif
