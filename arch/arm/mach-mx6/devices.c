/*
 * Copyright (C) 2011-2012 Freescale Semiconductor, Inc. All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.

 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/dma-mapping.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/ipu.h>
#include <linux/fb.h>
#include <linux/delay.h>
#include <linux/uio_driver.h>
#include <linux/iram_alloc.h>
#include <linux/fsl_devices.h>
#include <mach/common.h>
#include <mach/hardware.h>
#include <mach/gpio.h>

static struct mxc_gpio_port mxc_gpio_ports[] = {
	{
		.chip.label = "gpio-0",
		.base = IO_ADDRESS(GPIO1_BASE_ADDR),
		.irq = MXC_INT_GPIO1_INT15_0_NUM,
		.irq_high = MXC_INT_GPIO1_INT31_16_NUM,
		.virtual_irq_start = MXC_GPIO_IRQ_START
	},
	{
		.chip.label = "gpio-1",
		.base = IO_ADDRESS(GPIO2_BASE_ADDR),
		.irq = MXC_INT_GPIO2_INT15_0_NUM,
		.irq_high = MXC_INT_GPIO2_INT31_16_NUM,
		.virtual_irq_start = MXC_GPIO_IRQ_START + 32 * 1
	},
	{
		.chip.label = "gpio-2",
		.base = IO_ADDRESS(GPIO3_BASE_ADDR),
		.irq = MXC_INT_GPIO3_INT15_0_NUM,
		.irq_high = MXC_INT_GPIO3_INT31_16_NUM,
		.virtual_irq_start = MXC_GPIO_IRQ_START + 32 * 2
	},
	{
		.chip.label = "gpio-3",
		.base = IO_ADDRESS(GPIO4_BASE_ADDR),
		.irq = MXC_INT_GPIO4_INT15_0_NUM,
		.irq_high = MXC_INT_GPIO4_INT31_16_NUM,
		.virtual_irq_start = MXC_GPIO_IRQ_START + 32 * 3
	},
	{
		.chip.label = "gpio-4",
		.base = IO_ADDRESS(GPIO5_BASE_ADDR),
		.irq = MXC_INT_GPIO5_INT15_0_NUM,
		.irq_high = MXC_INT_GPIO5_INT31_16_NUM,
		.virtual_irq_start = MXC_GPIO_IRQ_START + 32 * 4
	},
	{
		.chip.label = "gpio-5",
		.base = IO_ADDRESS(GPIO6_BASE_ADDR),
		.irq = MXC_INT_GPIO6_INT15_0_NUM,
		.irq_high = MXC_INT_GPIO6_INT31_16_NUM,
		.virtual_irq_start = MXC_GPIO_IRQ_START + 32 * 5
	},
	{
		.chip.label = "gpio-6",
		.base = IO_ADDRESS(GPIO7_BASE_ADDR),
		.irq = MXC_INT_GPIO7_INT15_0_NUM,
		.irq_high = MXC_INT_GPIO7_INT31_16_NUM,
		.virtual_irq_start = MXC_GPIO_IRQ_START + 32 * 6
	},
};

int mx6q_register_gpios(void)
{
	/* 7 ports for Mx6 */
	return mxc_gpio_init(mxc_gpio_ports, 7);
}
