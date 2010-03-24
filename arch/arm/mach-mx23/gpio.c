/*
 * Copyright (C) 2009-2010 Freescale Semiconductor, Inc. All Rights Reserved.
 *
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
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/gpio.h>

#include <mach/pinctrl.h>

#include "regs-pinctrl.h"

#define PINCTRL_BASE_ADDR IO_ADDRESS(PINCTRL_PHYS_ADDR)

static int
mx23_gpio_direction(struct mxs_gpio_port *port, int pin, unsigned int input)
{
	void __iomem *base = PINCTRL_BASE_ADDR + 0x10 * port->id;
	if (input)
		__raw_writel(1 << pin, base + HW_PINCTRL_DOE0_CLR);
	else
		__raw_writel(1 << pin, base + HW_PINCTRL_DOE0_SET);

	return 0;
}

static int mx23_gpio_get(struct mxs_gpio_port *port, int pin)
{
	unsigned int data;
	void __iomem *base = PINCTRL_BASE_ADDR + 0x10 * port->id;
	data = __raw_readl(base + HW_PINCTRL_DIN0);
	return data & (1 << pin);
}

static void mx23_gpio_set(struct mxs_gpio_port *port, int pin, int data)
{
	void __iomem *base = PINCTRL_BASE_ADDR + 0x10 * port->id;
	if (data)
		__raw_writel(1 << pin, base + HW_PINCTRL_DOUT0_SET);
	else
		__raw_writel(1 << pin, base + HW_PINCTRL_DOUT0_CLR);
}

static unsigned int mx23_gpio_irq_stat(struct mxs_gpio_port *port)
{
	unsigned int mask;
	void __iomem *base = PINCTRL_BASE_ADDR + 0x10 * port->id;
	mask = __raw_readl(base + HW_PINCTRL_IRQSTAT0);
	mask &= __raw_readl(base + HW_PINCTRL_IRQEN0);
	return mask;
}

static int
mx23_gpio_set_irq_type(struct mxs_gpio_port *port, int pin, unsigned int type)
{
	unsigned int level, pol;
	void __iomem *base = PINCTRL_BASE_ADDR + 0x10 * port->id;
	switch (type) {
	case IRQ_TYPE_EDGE_RISING:
		level = 0;
		pol = 1;
		break;
	case IRQ_TYPE_EDGE_FALLING:
		level = 0;
		pol = 0;
		break;
	case IRQ_TYPE_LEVEL_HIGH:
		level = 1;
		pol = 1;
		break;
	case IRQ_TYPE_LEVEL_LOW:
		level = 1;
		pol = 0;
		break;
	default:
		pr_debug("%s: Incorrect GPIO interrupt type 0x%x\n",
			 __func__, type);
		return -ENXIO;
	}

	if (level)
		__raw_writel(1 << pin, base + HW_PINCTRL_IRQLEVEL0_SET);
	else
		__raw_writel(1 << pin, base + HW_PINCTRL_IRQLEVEL0_CLR);

	if (pol)
		__raw_writel(1 << pin, base + HW_PINCTRL_IRQPOL0_SET);
	else
		__raw_writel(1 << pin, base + HW_PINCTRL_IRQPOL0_CLR);

	return 0;
}

static void mx23_gpio_unmask_irq(struct mxs_gpio_port *port, int pin)
{
	void __iomem *base = PINCTRL_BASE_ADDR + 0x10 * port->id;
	__raw_writel(1 << pin, base + HW_PINCTRL_IRQEN0_SET);
}

static void mx23_gpio_mask_irq(struct mxs_gpio_port *port, int pin)
{
	void __iomem *base = PINCTRL_BASE_ADDR + 0x10 * port->id;
	__raw_writel(1 << pin, base + HW_PINCTRL_IRQEN0_CLR);
}

static void mx23_gpio_ack_irq(struct mxs_gpio_port *port, int pin)
{
	unsigned int mask;
	void __iomem *base = PINCTRL_BASE_ADDR + 0x10 * port->id;
	mask = 1 << pin;
	if (mask)
		__raw_writel(mask, base + HW_PINCTRL_IRQSTAT0_CLR);
}

static struct mxs_gpio_port mx23_gpios[] = {
	{
	 .irq = IRQ_GPIO0,
	 },
	{
	 .irq = IRQ_GPIO1,
	 },
	{
	 .irq = IRQ_GPIO2,
	 },
};

static struct mxs_gpio_chip mx23_gpio_chip = {
	.set_dir = mx23_gpio_direction,
	.get = mx23_gpio_get,
	.set = mx23_gpio_set,
	.get_irq_stat = mx23_gpio_irq_stat,
	.set_irq_type = mx23_gpio_set_irq_type,
	.unmask_irq = mx23_gpio_unmask_irq,
	.mask_irq = mx23_gpio_mask_irq,
	.ack_irq = mx23_gpio_ack_irq,
};

int __init mx23_gpio_init(void)
{
	int i;
	unsigned int reg;
	if (__raw_readl(PINCTRL_BASE_ADDR + HW_PINCTRL_CTRL_CLR) &
	    BM_PINCTRL_CTRL_SFTRST) {
		__raw_writel(BM_PINCTRL_CTRL_SFTRST,
			     PINCTRL_BASE_ADDR + HW_PINCTRL_CTRL_CLR);
		for (i = 0; i < 10000; i++) {
			if (!(__raw_readl(PINCTRL_BASE_ADDR + HW_PINCTRL_CTRL) &
			      BM_PINCTRL_CTRL_SFTRST))
				break;
			udelay(2);
		}
		if (i >= 10000)
			return -EFAULT;

		__raw_writel(BM_PINCTRL_CTRL_CLKGATE,
			     PINCTRL_BASE_ADDR + HW_PINCTRL_CTRL_CLR);
	}

	reg = __raw_readl(PINCTRL_BASE_ADDR + HW_PINCTRL_CTRL);
	for (i = 0; i < ARRAY_SIZE(mx23_gpios); i++) {
		void __iomem *base = PINCTRL_BASE_ADDR + 0x10 * i;
		if (!(reg & (BM_PINCTRL_CTRL_PRESENT0 << i)))
			continue;
		mxs_set_gpio_chip(&mx23_gpios[i], &mx23_gpio_chip);
		mx23_gpios[i].id = i;
		__raw_writel(0, base + HW_PINCTRL_IRQEN0);
		__raw_writel(0xFFFFFFFF, base + HW_PINCTRL_PIN2IRQ0);
		mx23_gpios[i].child_irq = MXS_GPIO_IRQ_START +
		    (i * PINS_PER_BANK);
		mxs_add_gpio_port(&mx23_gpios[i]);
	}
	return 0;
}
