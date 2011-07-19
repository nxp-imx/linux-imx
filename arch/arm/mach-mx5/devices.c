/*
 * Copyright 2009 Amit Kucheria <amit.kucheria@canonical.com>
 * Copyright (C) 2010-2011 Freescale Semiconductor, Inc.
 *
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <linux/gpio.h>
#include <mach/hardware.h>
#include <mach/imx-uart.h>
#include <mach/irqs.h>

static struct resource mxc_hsi2c_resources[] = {
	{
		.start = MX51_HSI2C_DMA_BASE_ADDR,
		.end = MX51_HSI2C_DMA_BASE_ADDR + SZ_16K - 1,
		.flags = IORESOURCE_MEM,
	},
	{
		.start = MX51_MXC_INT_HS_I2C,
		.end = MX51_MXC_INT_HS_I2C,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device mxc_hsi2c_device = {
	.name = "imx-i2c",
	.id = 2,
	.num_resources = ARRAY_SIZE(mxc_hsi2c_resources),
	.resource = mxc_hsi2c_resources
};

struct platform_device mxc_pm_device = {
	.name = "mx5_pm",
	.id = 0,
};

static u64 usb_dma_mask = DMA_BIT_MASK(32);

static struct resource usbotg_udc_resources[] = {
	{
		.start = MX51_OTG_BASE_ADDR,
		.end = MX51_OTG_BASE_ADDR + 0x1ff,
		.flags = IORESOURCE_MEM,
	},
	{
		.start = MX51_MXC_INT_USB_OTG,
		.flags = IORESOURCE_IRQ,
	},
};

/* OTG gadget device */
struct platform_device mxc_usbdr_udc_device = {
	.name		= "fsl-usb2-udc",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(usbotg_udc_resources),
	.resource	= usbotg_udc_resources,
	.dev		= {
		.dma_mask		= &usb_dma_mask,
		.coherent_dma_mask	= DMA_BIT_MASK(32),
	},
};

static struct resource usbotg_host_resources[] = {
	{
		.start = MX51_OTG_BASE_ADDR,
		.end = MX51_OTG_BASE_ADDR + 0x1ff,
		.flags = IORESOURCE_MEM,
	},
	{
		.start = MX51_MXC_INT_USB_OTG,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device mxc_usbdr_host_device = {
	.name = "fsl-ehci",
	.id = 0,
	.num_resources = ARRAY_SIZE(usbotg_host_resources),
	.resource = usbotg_host_resources,
	.dev = {
		.dma_mask = &usb_dma_mask,
		.coherent_dma_mask = DMA_BIT_MASK(32),
	},
};

static struct resource usbotg_wakeup_resources[] = {
	{
		.start = MX51_MXC_INT_USB_OTG,/* wakeup irq */
		.flags = IORESOURCE_IRQ,
	},
	{
		.start = MX51_MXC_INT_USB_OTG,/* usb core irq */
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device mxc_usbdr_wakeup_device = {
	.name = "usb_wakeup",
	.id   = 0,
	.num_resources = ARRAY_SIZE(usbotg_wakeup_resources),
	.resource = usbotg_wakeup_resources,
};

static struct resource usbotg_xcvr_resources[] = {
	{
		.start = MX51_OTG_BASE_ADDR,
		.end = MX51_OTG_BASE_ADDR + 0x1ff,
		.flags = IORESOURCE_MEM,
	},
	{
		.start = MX51_MXC_INT_USB_OTG,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device mxc_usbdr_otg_device = {
	.name = "fsl-usb2-otg",
	.id = -1,
	.dev		= {
		.dma_mask		= &usb_dma_mask,
		.coherent_dma_mask	= DMA_BIT_MASK(32),
	},
	.resource      = usbotg_xcvr_resources,
	.num_resources = ARRAY_SIZE(usbotg_xcvr_resources),
};

static struct resource usbh1_resources[] = {
	{
		.start = MX51_OTG_BASE_ADDR + 0x200,
		.end = MX51_OTG_BASE_ADDR + 0x200 + 0x1ff,
		.flags = IORESOURCE_MEM,
	},
	{
		.start = MX51_MXC_INT_USB_H1,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device mxc_usbh1_device = {
	.name = "fsl-ehci",
	.id = 1,
	.num_resources = ARRAY_SIZE(usbh1_resources),
	.resource = usbh1_resources,
	.dev = {
		.dma_mask = &usb_dma_mask,
		.coherent_dma_mask = DMA_BIT_MASK(32),
	},
};

static struct resource usbh1_wakeup_resources[] = {
	{
		.start = MX51_MXC_INT_USB_H1, /*wakeup irq*/
		.flags = IORESOURCE_IRQ,
	},
	{
		.start = MX51_MXC_INT_USB_H1,
		.flags = IORESOURCE_IRQ,/* usb core irq */
	},
};

struct platform_device mxc_usbh1_wakeup_device = {
	.name = "usb_wakeup",
	.id   = 1,
	.num_resources = ARRAY_SIZE(usbh1_wakeup_resources),
	.resource = usbh1_wakeup_resources,
};

static struct resource usbh2_resources[] = {
	{
		.start = MX51_OTG_BASE_ADDR + 0x400,
		.end = MX51_OTG_BASE_ADDR + 0x400 + 0x1ff,
		.flags = IORESOURCE_MEM,
	},
	{
		.start = MX51_MXC_INT_USB_H2,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device mxc_usbh2_device = {
	.name = "fsl-ehci",
	.id = 2,
	.num_resources = ARRAY_SIZE(usbh2_resources),
	.resource = usbh2_resources,
	.dev = {
		.dma_mask = &usb_dma_mask,
		.coherent_dma_mask = DMA_BIT_MASK(32),
	},
};

static struct resource usbh2_wakeup_resources[] = {
	{
		.start = MX51_MXC_INT_USB_H2,
		.flags = IORESOURCE_IRQ,/* wakeup irq */
	},
	{
		.start = MX51_MXC_INT_USB_H2,
		.flags = IORESOURCE_IRQ,/* usb core irq */
	},
};

struct platform_device mxc_usbh2_wakeup_device = {
	.name = "usb_wakeup",
	.id   = 2,
	.num_resources = ARRAY_SIZE(usbh2_wakeup_resources),
	.resource = usbh2_wakeup_resources,
};

struct platform_device imx_ahci_device_hwmon = {
	.name		= "imx-ahci-hwmon",
	.id		= -1,
};

static struct mxc_gpio_port mxc_gpio_ports[] = {
	{
		.chip.label = "gpio-0",
		.base = MX51_IO_ADDRESS(MX51_GPIO1_BASE_ADDR),
		.irq = MX51_MXC_INT_GPIO1_LOW,
		.irq_high = MX51_MXC_INT_GPIO1_HIGH,
		.virtual_irq_start = MXC_GPIO_IRQ_START
	},
	{
		.chip.label = "gpio-1",
		.base = MX51_IO_ADDRESS(MX51_GPIO2_BASE_ADDR),
		.irq = MX51_MXC_INT_GPIO2_LOW,
		.irq_high = MX51_MXC_INT_GPIO2_HIGH,
		.virtual_irq_start = MXC_GPIO_IRQ_START + 32 * 1
	},
	{
		.chip.label = "gpio-2",
		.base = MX51_IO_ADDRESS(MX51_GPIO3_BASE_ADDR),
		.irq = MX51_MXC_INT_GPIO3_LOW,
		.irq_high = MX51_MXC_INT_GPIO3_HIGH,
		.virtual_irq_start = MXC_GPIO_IRQ_START + 32 * 2
	},
	{
		.chip.label = "gpio-3",
		.base = MX51_IO_ADDRESS(MX51_GPIO4_BASE_ADDR),
		.irq = MX51_MXC_INT_GPIO4_LOW,
		.irq_high = MX51_MXC_INT_GPIO4_HIGH,
		.virtual_irq_start = MXC_GPIO_IRQ_START + 32 * 3
	},
	{
		.chip.label = "gpio-4",
		.base = MX53_IO_ADDRESS(MX53_GPIO5_BASE_ADDR),
		.irq = MX53_INT_GPIO5_LOW,
		.irq_high = MX53_INT_GPIO5_HIGH,
		.virtual_irq_start = MXC_GPIO_IRQ_START + 32 * 4
	},
	{
		.chip.label = "gpio-5",
		.base = MX53_IO_ADDRESS(MX53_GPIO6_BASE_ADDR),
		.irq = MX53_INT_GPIO6_LOW,
		.irq_high = MX53_INT_GPIO6_HIGH,
		.virtual_irq_start = MXC_GPIO_IRQ_START + 32 * 5
	},
	{
		.chip.label = "gpio-6",
		.base = MX53_IO_ADDRESS(MX53_GPIO7_BASE_ADDR),
		.irq = MX53_INT_GPIO7_LOW,
		.irq_high = MX53_INT_GPIO7_HIGH,
		.virtual_irq_start = MXC_GPIO_IRQ_START + 32 * 6
	},
};

int __init imx51_register_gpios(void)
{
	return mxc_gpio_init(mxc_gpio_ports, 4);
}

int __init imx53_register_gpios(void)
{
	return mxc_gpio_init(mxc_gpio_ports, ARRAY_SIZE(mxc_gpio_ports));
}

