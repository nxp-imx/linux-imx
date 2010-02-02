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
#include <linux/platform_device.h>

#include <mach/pinctrl.h>

#include "regs-pinctrl.h"

#define PINCTRL_BASE_ADDR IO_ADDRESS(PINCTRL_PHYS_ADDR)

static int
mx23_pin2id(struct pinctrl_chip *chip, unsigned int pin, unsigned int *id)
{
	int bank;
	bank = MXS_PIN_TO_BANK(pin & MXS_GPIO_MASK);
	if (bank == MXS_PIN_BANK_MAX)
		return -EINVAL;
	*id = MXS_PIN_TO_PINID(pin & MXS_GPIO_MASK);
	return bank;
}

static unsigned int mx23_get_gpio(struct pin_bank *bank, unsigned int id)
{
	if (bank->gpio_port >= MXS_NON_GPIO)
		return -EINVAL;
	return bank->gpio_port * PINS_PER_BANK + id;
}

static void mx23_set_strength(struct pin_bank *bank,
			      unsigned int id, enum pad_strength strength)
{
	void __iomem *addr;
	addr = PINCTRL_BASE_ADDR + HW_PINCTRL_DRIVE0;
	addr += 0x40 * bank->id + 0x10 * (id >> 3);
	id &= 0x7;
	id *= 4;
	__raw_writel(PAD_CLEAR << id, addr + CLR_REGISTER);
	__raw_writel(strength << id, addr + SET_REGISTER);
}

static void mx23_set_voltage(struct pin_bank *bank,
			     unsigned int id, enum pad_voltage volt)
{
	void __iomem *addr;
	addr = PINCTRL_BASE_ADDR + HW_PINCTRL_DRIVE0;
	addr += 0x40 * bank->id + 0x10 * (id >> 3);
	id &= 0x7;
	id = id * 4 + 2;
	if (volt == PAD_1_8V)
		__raw_writel(1 << id, addr + CLR_REGISTER);
	else
		__raw_writel(1 << id, addr + SET_REGISTER);
}

static void mx23_set_pullup(struct pin_bank *bank, unsigned int id, int pullup)
{
	void __iomem *addr;
	addr = PINCTRL_BASE_ADDR + HW_PINCTRL_PULL0;
	addr += 0x10 * bank->id;
	if (pullup)
		__raw_writel(1 << id, addr + SET_REGISTER);
	else
		__raw_writel(1 << id, addr + CLR_REGISTER);
}

static void mx23_set_type(struct pin_bank *bank,
			  unsigned int id, enum pin_fun cfg)
{
	void __iomem *addr;
	addr = PINCTRL_BASE_ADDR + HW_PINCTRL_MUXSEL0;
	addr += 0x20 * bank->id + 0x10 * (id >> 4);
	id &= 0xF;
	id *= 2;
	__raw_writel(0x3 << id, addr + CLR_REGISTER);
	__raw_writel(cfg << id, addr + SET_REGISTER);
}

static struct pin_bank mx23_pin_banks[6] = {
	[0] = {
	       .id = 0,
	       .gpio_port = 0,
	       },
	[1] = {
	       .id = 1,
	       .gpio_port = 1,
	       },
	[2] = {
	       .id = 2,
	       .gpio_port = 2,
	       },
	[3] = {
	       .id = 3,
	       .gpio_port = 3,
	       },
	[4] = {
	       .id = 4,
	       .gpio_port = 4,
	       },
	[5] = {
	       .id = 5,
	       .gpio_port = MXS_NON_GPIO,
	       }
};

static struct pinctrl_chip mx23_pinctrl = {
	.name = "pinctrl",
	.banks = mx23_pin_banks,
	.pin2id = mx23_pin2id,
	.get_gpio = mx23_get_gpio,
	.set_strength = mx23_set_strength,
	.set_voltage = mx23_set_voltage,
	.set_pullup = mx23_set_pullup,
	.set_type = mx23_set_type,
};

int __init mx23_pinctrl_init(void)
{
	int i;
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

	__raw_writel(BM_PINCTRL_CTRL_CLKGATE,
		     PINCTRL_BASE_ADDR + HW_PINCTRL_CTRL_CLR);
	mx23_pinctrl.bank_size = ARRAY_SIZE(mx23_pin_banks);
	return mxs_set_pinctrl_chip(&mx23_pinctrl);
}
