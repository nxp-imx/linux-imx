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

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/gpio.h>
#include <linux/list.h>
#include <linux/irq.h>

#include <mach/hardware.h>
#include <mach/pinctrl.h>

#if MXS_ARCH_NR_GPIOS % PINS_PER_BANK
#error "MXS_ARCH_NR_GPIOS must be multipled of PINS_PER_BANK"
#endif

static struct mxs_gpio_port *mxs_gpios[MXS_ARCH_NR_GPIOS / PINS_PER_BANK];

static inline int mxs_valid_gpio(struct mxs_gpio_port *port)
{
	struct mxs_gpio_chip *chip = port->chip;

	if (port->id >= (MXS_ARCH_NR_GPIOS / PINS_PER_BANK))
		return -EINVAL;

	if (port->irq < 0 && port->child_irq > 0)
		return -EINVAL;
	if (chip->get == NULL || chip->set == NULL || chip->set_dir == NULL)
		return -EINVAL;
	if (port->child_irq > 0) {
		if (chip->get_irq_stat == NULL)
			return -EINVAL;
		if (chip->mask_irq == NULL || chip->unmask_irq == NULL)
			return -EINVAL;
	}
	return 0;
}

static int mxs_gpio_request(struct gpio_chip *chip, unsigned int pin)
{
	struct mxs_gpio_port *port;
	port = container_of(chip, struct mxs_gpio_port, port);
	return mxs_request_pin(MXS_PIN_ENCODE(port->id, pin),
			       PIN_GPIO, GPIO_ID_NAME);
}

static void mxs_gpio_free(struct gpio_chip *chip, unsigned int pin)
{
	struct mxs_gpio_port *port;
	port = container_of(chip, struct mxs_gpio_port, port);
	return mxs_release_pin(MXS_PIN_ENCODE(port->id, pin), GPIO_ID_NAME);
}

static int mxs_gpio_to_irq(struct gpio_chip *chip, unsigned int index)
{
	struct mxs_gpio_port *port;
	port = container_of(chip, struct mxs_gpio_port, port);
	if (port->child_irq < 0)
		return -ENXIO;
	return port->child_irq + index;
}

static int mxs_gpio_get(struct gpio_chip *chip, unsigned int index)
{
	struct mxs_gpio_port *port;
	port = container_of(chip, struct mxs_gpio_port, port);
	return port->chip->get(port, index);
}

static void mxs_gpio_set(struct gpio_chip *chip, unsigned int index, int v)
{
	struct mxs_gpio_port *port;
	port = container_of(chip, struct mxs_gpio_port, port);
	port->chip->set(port, index, v);
}

static int mxs_gpio_output(struct gpio_chip *chip, unsigned int index, int v)
{
	int ret;
	struct mxs_gpio_port *port;
	port = container_of(chip, struct mxs_gpio_port, port);
	ret = port->chip->set_dir(port, index, 0);
	if (!ret)
		port->chip->set(port, index, v);
	return ret;
}

static int mxs_gpio_input(struct gpio_chip *chip, unsigned int index)
{
	struct mxs_gpio_port *port;
	port = container_of(chip, struct mxs_gpio_port, port);
	return port->chip->set_dir(port, index, 1);
}

static void mxs_gpio_irq_handler(u32 irq, struct irq_desc *desc)
{
	struct mxs_gpio_port *port = get_irq_data(irq);
	int gpio_irq = port->child_irq;
	u32 irq_stat = port->chip->get_irq_stat(port);

	desc->chip->mask(irq);

	while (irq_stat) {
		if (irq_stat & 1)
			generic_handle_irq(gpio_irq);
		gpio_irq++;
		irq_stat >>= 1;
	}

	desc->chip->ack(irq);
	desc->chip->unmask(irq);
}

static int mxs_gpio_set_irq_type(unsigned int irq, unsigned int type)
{
	struct mxs_gpio_port *port;
	unsigned int gpio = irq_to_gpio(irq);
	port = mxs_gpios[GPIO_TO_BANK(gpio)];
	if (port->child_irq < 0)
		return -ENXIO;
	if (port->chip->set_irq_type)
		return port->chip->set_irq_type(port, GPIO_TO_PINS(gpio), type);
	return -ENODEV;
}

static void mxs_gpio_ack_irq(unsigned int irq)
{
	struct mxs_gpio_port *port;
	unsigned int gpio = irq_to_gpio(irq);
	port = mxs_gpios[GPIO_TO_BANK(gpio)];
	if (port->child_irq < 0)
		return;
	if (port->chip->ack_irq)
		port->chip->ack_irq(port, GPIO_TO_PINS(gpio));
}

static void mxs_gpio_mask_irq(unsigned int irq)
{
	struct mxs_gpio_port *port;
	unsigned int gpio = irq_to_gpio(irq);
	port = mxs_gpios[GPIO_TO_BANK(gpio)];
	if (port->child_irq < 0)
		return;
	port->chip->mask_irq(port, GPIO_TO_PINS(gpio));
}

static void mxs_gpio_unmask_irq(unsigned int irq)
{
	struct mxs_gpio_port *port;
	unsigned int gpio = irq_to_gpio(irq);
	port = mxs_gpios[GPIO_TO_BANK(gpio)];
	if (port->child_irq < 0)
		return;
	port->chip->unmask_irq(port, GPIO_TO_PINS(gpio));
}

static struct irq_chip gpio_irq_chip = {
	.ack = mxs_gpio_ack_irq,
	.mask = mxs_gpio_mask_irq,
	.unmask = mxs_gpio_unmask_irq,
	.enable = mxs_gpio_unmask_irq,
	.disable = mxs_gpio_mask_irq,
	.set_type = mxs_gpio_set_irq_type,
};

int __init mxs_add_gpio_port(struct mxs_gpio_port *port)
{
	int i, ret;
	if (!(port && port->chip))
		return -EINVAL;

	if (mxs_valid_gpio(port))
		return -EINVAL;

	if (mxs_gpios[port->id])
		return -EBUSY;

	mxs_gpios[port->id] = port;

	port->port.base = port->id * PINS_PER_BANK;
	port->port.ngpio = PINS_PER_BANK;
	port->port.can_sleep = 1;
	port->port.exported = 1;
	port->port.to_irq = mxs_gpio_to_irq;
	port->port.direction_input = mxs_gpio_input;
	port->port.direction_output = mxs_gpio_output;
	port->port.get = mxs_gpio_get;
	port->port.set = mxs_gpio_set;
	port->port.request = mxs_gpio_request;
	port->port.free = mxs_gpio_free;
	port->port.owner = THIS_MODULE;
	ret = gpiochip_add(&port->port);
	if (ret < 0)
		return ret;

	if (port->child_irq < 0)
		return 0;

	for (i = 0; i < PINS_PER_BANK; i++) {
		gpio_irq_chip.mask(port->child_irq + i);
		set_irq_chip(port->child_irq + i, &gpio_irq_chip);
		set_irq_handler(port->child_irq + i, handle_level_irq);
		set_irq_flags(port->child_irq + i, IRQF_VALID);
	}
	set_irq_chained_handler(port->irq, mxs_gpio_irq_handler);
	set_irq_data(port->irq, port);
	return ret;
};
