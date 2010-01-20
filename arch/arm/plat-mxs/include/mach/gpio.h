/*
 * Copyright (C) 1999 ARM Limited
 * Copyright (C) 2009-2010 Freescale Semiconductor, Inc.
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

#ifndef __ASM_ARCH_GPIO_H__
#define __ASM_ARCH_GPIO_H__

#include <mach/hardware.h>
#include <asm-generic/gpio.h>

#define	GPIO_ID_NAME		"gpio"
/* use gpiolib dispatchers */
#define gpio_get_value          __gpio_get_value
#define gpio_set_value          __gpio_set_value
#define gpio_cansleep           __gpio_cansleep
#define gpio_to_irq		__gpio_to_irq
#define irq_to_gpio(irq)	((irq) - MXS_GPIO_IRQ_START)

struct mxs_gpio_port;
struct mxs_gpio_chip {
	int (*set_dir) (struct mxs_gpio_port *, int, unsigned int);
	int (*get) (struct mxs_gpio_port *, int);
	void (*set) (struct mxs_gpio_port *, int, int);
	unsigned int (*get_irq_stat) (struct mxs_gpio_port *);
	int (*set_irq_type) (struct mxs_gpio_port *, int, unsigned int);
	void (*unmask_irq) (struct mxs_gpio_port *, int);
	void (*mask_irq) (struct mxs_gpio_port *, int);
	void (*ack_irq) (struct mxs_gpio_port *, int);
};

struct mxs_gpio_port {
	int id;
	int irq;
	int child_irq;
	struct mxs_gpio_chip *chip;
	struct gpio_chip port;
};

extern int mxs_add_gpio_port(struct mxs_gpio_port *port);

static inline void
mxs_set_gpio_chip(struct mxs_gpio_port *port, struct mxs_gpio_chip *chip)
{
	if (port && chip)
		port->chip = chip;
}

#endif /* __ASM_ARCH_GPIO_H__ */
