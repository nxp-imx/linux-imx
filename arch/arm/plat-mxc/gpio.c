/*
 * MXC GPIO support. (c) 2008 Daniel Mack <daniel@caiaq.de>
 * Copyright 2008 Juergen Beisert, kernel@pengutronix.de
 *
 * Based on code from Freescale,
 * Copyright (C) 2004-2010 Freescale Semiconductor, Inc. All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/sysdev.h>
#include <mach/hardware.h>
#include <asm-generic/bug.h>

static struct mxc_gpio_port *mxc_gpio_ports;
static int gpio_table_size;

#define cpu_is_mx1_mx2()	(cpu_is_mx1() || cpu_is_mx2())

#define GPIO_DR		(cpu_is_mx1_mx2() ? 0x1c : 0x00)
#define GPIO_GDIR	(cpu_is_mx1_mx2() ? 0x00 : 0x04)
#define GPIO_PSR	(cpu_is_mx1_mx2() ? 0x24 : 0x08)
#define GPIO_ICR1	(cpu_is_mx1_mx2() ? 0x28 : 0x0C)
#define GPIO_ICR2	(cpu_is_mx1_mx2() ? 0x2C : 0x10)
#define GPIO_IMR	(cpu_is_mx1_mx2() ? 0x30 : 0x14)
#define GPIO_ISR	(cpu_is_mx1_mx2() ? 0x34 : 0x18)

#define GPIO_INT_LOW_LEV	(cpu_is_mx1_mx2() ? 0x3 : 0x0)
#define GPIO_INT_HIGH_LEV	(cpu_is_mx1_mx2() ? 0x2 : 0x1)
#define GPIO_INT_RISE_EDGE	(cpu_is_mx1_mx2() ? 0x0 : 0x2)
#define GPIO_INT_FALL_EDGE	(cpu_is_mx1_mx2() ? 0x1 : 0x3)
#define GPIO_INT_NONE		0x4

/* Note: This driver assumes 32 GPIOs are handled in one register */

static void _clear_gpio_irqstatus(struct mxc_gpio_port *port, u32 index)
{
	__raw_writel(1 << index, port->base + GPIO_ISR);
}

static void _set_gpio_irqenable(struct mxc_gpio_port *port, u32 index,
				int enable)
{
	u32 l;

	l = __raw_readl(port->base + GPIO_IMR);
	l = (l & (~(1 << index))) | (!!enable << index);
	__raw_writel(l, port->base + GPIO_IMR);
}

static void gpio_ack_irq(u32 irq)
{
	u32 gpio = irq_to_gpio(irq);
	_clear_gpio_irqstatus(&mxc_gpio_ports[gpio / 32], gpio & 0x1f);
}

static void gpio_mask_irq(u32 irq)
{
	u32 gpio = irq_to_gpio(irq);
	_set_gpio_irqenable(&mxc_gpio_ports[gpio / 32], gpio & 0x1f, 0);
}

static void gpio_unmask_irq(u32 irq)
{
	u32 gpio = irq_to_gpio(irq);
	_set_gpio_irqenable(&mxc_gpio_ports[gpio / 32], gpio & 0x1f, 1);
}

static int mxc_gpio_get(struct gpio_chip *chip, unsigned offset);

static int gpio_set_irq_type(u32 irq, u32 type)
{
	u32 gpio = irq_to_gpio(irq);
	struct mxc_gpio_port *port = &mxc_gpio_ports[gpio / 32];
	u32 bit, val;
	int edge;
	void __iomem *reg = port->base;

	port->both_edges &= ~(1 << (gpio & 31));
	switch (type) {
	case IRQ_TYPE_EDGE_RISING:
		edge = GPIO_INT_RISE_EDGE;
		break;
	case IRQ_TYPE_EDGE_FALLING:
		edge = GPIO_INT_FALL_EDGE;
		break;
	case IRQ_TYPE_EDGE_BOTH:
		val = mxc_gpio_get(&port->chip, gpio & 31);
		if (val) {
			edge = GPIO_INT_LOW_LEV;
			pr_debug("mxc: set GPIO %d to low trigger\n", gpio);
		} else {
			edge = GPIO_INT_HIGH_LEV;
			pr_debug("mxc: set GPIO %d to high trigger\n", gpio);
		}
		port->both_edges |= 1 << (gpio & 31);
		break;
	case IRQ_TYPE_LEVEL_LOW:
		edge = GPIO_INT_LOW_LEV;
		break;
	case IRQ_TYPE_LEVEL_HIGH:
		edge = GPIO_INT_HIGH_LEV;
		break;
	default:
		return -EINVAL;
	}

	/* set the correct irq handler */
	if (type & IRQ_TYPE_EDGE_BOTH)
		set_irq_handler(irq, handle_edge_irq);
	else if (type & (IRQ_TYPE_LEVEL_LOW | IRQ_TYPE_LEVEL_HIGH))
		set_irq_handler(irq, handle_level_irq);

	reg += GPIO_ICR1 + ((gpio & 0x10) >> 2); /* lower or upper register */
	bit = gpio & 0xf;
	val = __raw_readl(reg) & ~(0x3 << (bit << 1));
	__raw_writel(val | (edge << (bit << 1)), reg);
	_clear_gpio_irqstatus(port, gpio & 0x1f);

	return 0;
}

static void mxc_flip_edge(struct mxc_gpio_port *port, u32 gpio)
{
	void __iomem *reg = port->base;
	u32 bit, val;
	int edge;

	reg += GPIO_ICR1 + ((gpio & 0x10) >> 2); /* lower or upper register */
	bit = gpio & 0xf;
	val = __raw_readl(reg);
	edge = (val >> (bit << 1)) & 3;
	val &= ~(0x3 << (bit << 1));
	if (edge == GPIO_INT_HIGH_LEV) {
		edge = GPIO_INT_LOW_LEV;
		pr_debug("mxc: switch GPIO %d to low trigger\n", gpio);
	} else if (edge == GPIO_INT_LOW_LEV) {
		edge = GPIO_INT_HIGH_LEV;
		pr_debug("mxc: switch GPIO %d to high trigger\n", gpio);
	} else {
		pr_err("mxc: invalid configuration for GPIO %d: %x\n",
		       gpio, edge);
		return;
	}
	__raw_writel(val | (edge << (bit << 1)), reg);
}

/* handle 32 interrupts in one status register */
static void mxc_gpio_irq_handler(struct mxc_gpio_port *port, u32 irq_stat)
{
	u32 gpio_irq_no_base = port->virtual_irq_start;

	while (irq_stat != 0) {
		int irqoffset = fls(irq_stat) - 1;

		if (port->both_edges & (1 << irqoffset))
			mxc_flip_edge(port, irqoffset);

		generic_handle_irq(gpio_irq_no_base + irqoffset);

		irq_stat &= ~(1 << irqoffset);
	}
}

/* MX1 and MX3 has one interrupt *per* gpio port */
static void mx3_gpio_irq_handler(u32 irq, struct irq_desc *desc)
{
	u32 irq_stat;
	u32 mask = 0xFFFFFFFF;
	struct mxc_gpio_port *port = (struct mxc_gpio_port *)get_irq_data(irq);

	if (port->irq_high) {
		if (irq == port->irq)
			mask = 0x0000FFFF;
		else
			mask = 0xFFFF0000;
	}

	irq_stat = __raw_readl(port->base + GPIO_ISR) &
			(__raw_readl(port->base + GPIO_IMR) & mask);
	mxc_gpio_irq_handler(port, irq_stat);
}

/* MX2 has one interrupt *for all* gpio ports */
static void mx2_gpio_irq_handler(u32 irq, struct irq_desc *desc)
{
	int i;
	u32 irq_msk, irq_stat;
	struct mxc_gpio_port *port = (struct mxc_gpio_port *)get_irq_data(irq);

	/* walk through all interrupt status registers */
	for (i = 0; i < gpio_table_size; i++) {
		irq_msk = __raw_readl(port[i].base + GPIO_IMR);
		if (!irq_msk)
			continue;

		irq_stat = __raw_readl(port[i].base + GPIO_ISR) & irq_msk;
		if (irq_stat)
			mxc_gpio_irq_handler(&port[i], irq_stat);
	}
}

/*
 * Set interrupt number "irq" in the GPIO as a wake-up source.
 * While system is running all registered GPIO interrupts need to have
 * wake-up enabled. When system is suspended, only selected GPIO interrupts
 * need to have wake-up enabled.
 * @param  irq          interrupt source number
 * @param  enable       enable as wake-up if equal to non-zero
 * @return       This function returns 0 on success.
 */
static int gpio_set_wake_irq(u32 irq, u32 enable)
{
	u32 gpio = irq_to_gpio(irq);
	u32 gpio_idx = gpio & 0x1F;
	struct mxc_gpio_port *port = &mxc_gpio_ports[gpio / 32];

	if (enable) {
		port->suspend_wakeup |= (1 << gpio_idx);
		if (port->irq_high && (gpio_idx >= 16))
			enable_irq_wake(port->irq_high);
		else
			enable_irq_wake(port->irq);
	} else {
		port->suspend_wakeup &= ~(1 << gpio_idx);
		if (port->irq_high && (gpio_idx >= 16))
			disable_irq_wake(port->irq_high);
		else
			disable_irq_wake(port->irq);
	}

	return 0;
}

static struct irq_chip gpio_irq_chip = {
	.ack = gpio_ack_irq,
	.mask = gpio_mask_irq,
	.unmask = gpio_unmask_irq,
	.set_type = gpio_set_irq_type,
	.set_wake = gpio_set_wake_irq,
};

static void _set_gpio_direction(struct gpio_chip *chip, unsigned offset,
				int dir)
{
	struct mxc_gpio_port *port =
		container_of(chip, struct mxc_gpio_port, chip);
	u32 l;
	unsigned long flags;

	spin_lock_irqsave(&port->lock, flags);
	l = __raw_readl(port->base + GPIO_GDIR);
	if (dir)
		l |= 1 << offset;
	else
		l &= ~(1 << offset);
	__raw_writel(l, port->base + GPIO_GDIR);
	spin_unlock_irqrestore(&port->lock, flags);
}

static void mxc_gpio_set(struct gpio_chip *chip, unsigned offset, int value)
{
	struct mxc_gpio_port *port =
		container_of(chip, struct mxc_gpio_port, chip);
	void __iomem *reg = port->base + GPIO_DR;
	u32 l;
	unsigned long flags;

	spin_lock_irqsave(&port->lock, flags);
	l = (__raw_readl(reg) & (~(1 << offset))) | (value << offset);
	__raw_writel(l, reg);
	spin_unlock_irqrestore(&port->lock, flags);
}

static int mxc_gpio_get(struct gpio_chip *chip, unsigned offset)
{
	struct mxc_gpio_port *port =
		container_of(chip, struct mxc_gpio_port, chip);

	return (__raw_readl(port->base + GPIO_PSR) >> offset) & 1;
}

static int mxc_gpio_direction_input(struct gpio_chip *chip, unsigned offset)
{
	_set_gpio_direction(chip, offset, 0);
	return 0;
}

static int mxc_gpio_direction_output(struct gpio_chip *chip,
				     unsigned offset, int value)
{
	mxc_gpio_set(chip, offset, value);
	_set_gpio_direction(chip, offset, 1);
	return 0;
}

int __init mxc_gpio_init(struct mxc_gpio_port *port, int cnt)
{
	int i, j;
	int ret = 0;

	/* save for local usage */
	mxc_gpio_ports = port;
	gpio_table_size = cnt;

	printk(KERN_INFO "MXC GPIO hardware\n");

	for (i = 0; i < cnt; i++) {
		/* disable the interrupt and clear the status */
		__raw_writel(0, port[i].base + GPIO_IMR);
		__raw_writel(~0, port[i].base + GPIO_ISR);
		for (j = port[i].virtual_irq_start;
			j < port[i].virtual_irq_start + 32; j++) {
			set_irq_chip(j, &gpio_irq_chip);
			set_irq_handler(j, handle_level_irq);
			set_irq_flags(j, IRQF_VALID);
		}

		/* register gpio chip */
		port[i].chip.direction_input = mxc_gpio_direction_input;
		port[i].chip.direction_output = mxc_gpio_direction_output;
		port[i].chip.get = mxc_gpio_get;
		port[i].chip.set = mxc_gpio_set;
		port[i].chip.base = i * 32;
		port[i].chip.ngpio = 32;

		spin_lock_init(&port[i].lock);

		/* its a serious configuration bug when it fails */
		BUG_ON( gpiochip_add(&port[i].chip) < 0 );

		if (!cpu_is_mx2() || cpu_is_mx25()) {
			/* setup one handler for each entry */
			set_irq_chained_handler(port[i].irq, mx3_gpio_irq_handler);
			set_irq_data(port[i].irq, &port[i]);
			if (port[i].irq_high) {
				set_irq_chained_handler(port[i].irq_high, mx3_gpio_irq_handler);
				set_irq_data(port[i].irq_high, &port[i]);
			}
		}
	}

	if (cpu_is_mx2()) {
		/* setup one handler for all GPIO interrupts */
		set_irq_chained_handler(port[0].irq, mx2_gpio_irq_handler);
		set_irq_data(port[0].irq, port);
	}

	return ret;
}

#ifdef CONFIG_PM
/*!
 * This function puts the GPIO in low-power mode/state.
 * All the interrupts that are enabled are first saved.
 * Only those interrupts which registers as a wake source by calling
 * enable_irq_wake are enabled. All other interrupts are disabled.
 *
 * @param   dev  the system device structure used to give information
 *                on GPIO to suspend
 * @param   mesg the power state the device is entering
 *
 * @return  The function always returns 0.
 */
static int mxc_gpio_suspend(struct sys_device *dev, pm_message_t mesg)
{
	int i;
	struct mxc_gpio_port *port = mxc_gpio_ports;

	for (i = 0; i < gpio_table_size; i++) {
		void __iomem *isr_reg;
		void __iomem *imr_reg;

		isr_reg = port[i].base + GPIO_ISR;
		imr_reg = port[i].base + GPIO_IMR;

		if (__raw_readl(isr_reg) & port[i].suspend_wakeup)
			return -EPERM;

		port[i].saved_wakeup = __raw_readl(imr_reg);
		__raw_writel(port[i].suspend_wakeup, imr_reg);
	}

	return 0;
}

/*!
 * This function brings the GPIO back from low-power state.
 * All the interrupts enabled before suspension are re-enabled from
 * the saved information.
 *
 * @param   dev  the system device structure used to give information
 *                on GPIO to resume
 *
 * @return  The function always returns 0.
 */
static int mxc_gpio_resume(struct sys_device *dev)
{
	int i;
	struct mxc_gpio_port *port = mxc_gpio_ports;

	for (i = 0; i < gpio_table_size; i++) {
		void __iomem *isr_reg;
		void __iomem *imr_reg;

		isr_reg = port[i].base + GPIO_ISR;
		imr_reg = port[i].base + GPIO_IMR;

		__raw_writel(port[i].saved_wakeup, imr_reg);
	}

	return 0;
}
#else
#define mxc_gpio_suspend  NULL
#define mxc_gpio_resume   NULL
#endif				/* CONFIG_PM */

/*!
 * This structure contains pointers to the power management callback functions.
 */
static struct sysdev_class mxc_gpio_sysclass = {
	.name = "mxc_gpio",
	.suspend = mxc_gpio_suspend,
	.resume = mxc_gpio_resume,
};

/*!
 * This structure represents GPIO as a system device.
 * System devices follow a slightly different driver model.
 * They don't need to do dynammic driver binding, can't be probed,
 * and don't reside on any type of peripheral bus.
 * So, it is represented and treated a little differently.
 */
static struct sys_device mxc_gpio_device = {
	.id = 0,
	.cls = &mxc_gpio_sysclass,
};

static int gpio_sysdev_init(void)
{
	int ret = sysdev_class_register(&mxc_gpio_sysclass);
	if (ret)
		return ret;
	return sysdev_register(&mxc_gpio_device);
}
arch_initcall(gpio_sysdev_init);

