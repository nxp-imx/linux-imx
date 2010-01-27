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

#ifndef __ASM_ARM_ARCH_PINCTRL_H
#define __ASM_ARM_ARCH_PINCTRL_H

#include <linux/types.h>
#include <linux/gpio.h>

#define PINS_PER_BANK		32
#define GPIO_TO_PINS(gpio)	((gpio) % 32)
#define GPIO_TO_BANK(gpio)	((gpio) / 32)

#define MXS_PIN_TO_GPIO(p)	(((p) & MXS_PIN_PINID_MAX) |\
				 ((((p) >> MXS_PIN_BANK_BIT) &\
				 MXS_PIN_BANK_MAX) * PINS_PER_BANK))

#define MXS_PIN_BANK_BIT	24
#define MXS_PIN_BANK_MAX	(0x7FFFFFFF >> (MXS_PIN_BANK_BIT - 1))
#define MXS_PIN_PINID_MAX	((1 << MXS_PIN_BANK_BIT) - 1)
#define MXS_PIN_TO_BANK(p)	(((p) >> MXS_PIN_BANK_BIT) & MXS_PIN_BANK_MAX)
#define MXS_PIN_TO_PINID(p)	((p) & MXS_PIN_PINID_MAX)

#define MXS_PIN_ENCODE(b, p)	\
		((((b) & MXS_PIN_BANK_MAX) << MXS_PIN_BANK_BIT) |\
		 ((p) & MXS_PIN_PINID_MAX))

#define MXS_GPIO_MASK		0x7FFFFFFF
#define MXS_NON_GPIO		0x80000000
/*
 * Each pin may be routed up to four different HW interfaces
 * including GPIO
 */
enum pin_fun {
	PIN_FUN1 = 0,
	PIN_FUN2,
	PIN_FUN3,
	PIN_GPIO,
};

/*
 * Each pin may have different output drive strength in range from
 * 4mA to 20mA. The most common case is 4, 8 and 12 mA strengths.
 */
enum pad_strength {
	PAD_4MA = 0,
	PAD_8MA,
	PAD_12MA,
	PAD_RESV,
	PAD_CLEAR = PAD_RESV,
};

/*
 * Each pin can be programmed for 1.8V or 3.3V
 */
enum pad_voltage {
	PAD_1_8V = 0,
	PAD_3_3V,
};

/*
 * Structure to define a group of pins and their parameters
 */
struct pin_desc {
	char *name;
	unsigned int id;
	enum pin_fun fun;
	enum pad_strength strength;
	enum pad_voltage voltage;
	unsigned pullup:1;
	unsigned drive:1;
	unsigned pull:1;
	unsigned input:1;
	unsigned data:1;
};

struct pin_bank {
	const char *label[sizeof(long) * 8];
	unsigned long id;
	struct pinctrl_chip *chip;
	unsigned long bitmap;
	unsigned long gpio_port;
};

struct pinctrl_chip {
	char *name;
	unsigned int nouse;
	unsigned int bank_size;
	struct pin_bank *banks;
	/* OPS */
	int (*pin2id) (struct pinctrl_chip *, unsigned int, unsigned int *);
	unsigned int (*get_gpio) (struct pin_bank *, unsigned int);
	void (*set_strength) (struct pin_bank *, unsigned int,
			      enum pad_strength);
	void (*set_voltage) (struct pin_bank *, unsigned int, enum pad_voltage);
	void (*set_pullup) (struct pin_bank *, unsigned int, int);
	void (*set_type) (struct pin_bank *, unsigned int, enum pin_fun);
};

extern int __init mxs_set_pinctrl_chip(struct pinctrl_chip *);

extern unsigned int mxs_pin2gpio(unsigned int);
extern int mxs_request_pin(unsigned int, enum pin_fun, const char *);
extern int mxs_set_type(unsigned int, enum pin_fun, const char *);
extern int mxs_set_strength(unsigned int, enum pad_strength, const char *);
extern int mxs_set_voltage(unsigned int, enum pad_voltage, const char *);
extern int mxs_set_pullup(unsigned int, int, const char *);
extern void mxs_release_pin(unsigned int, const char *);
#endif
