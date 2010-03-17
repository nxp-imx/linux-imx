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
#include <linux/bitops.h>
#include <linux/gpio.h>
#include <linux/irq.h>

#include <mach/pinctrl.h>

static struct pinctrl_chip *g_chip;

int mxs_request_pin(unsigned int pin, enum pin_fun fun, const char *lab)
{
	int bank, index;
	struct pin_bank *pb;
	if (g_chip == NULL)
		return -ENODEV;

	bank = g_chip->pin2id(g_chip, pin, &index);
	if (bank < 0 || index < 0 || bank >= g_chip->bank_size)
		return -EFAULT;

	pb = g_chip->banks + bank;
	if (test_and_set_bit(index, &pb->bitmap))
		return -EBUSY;
	pb->label[index] = lab;
	if (g_chip->set_type)
		g_chip->set_type(pb, index, fun);
	return 0;
}
EXPORT_SYMBOL(mxs_request_pin);

int mxs_get_type(unsigned int pin)
{
	int bank, index;
	struct pin_bank *pb;
	int ret = 0;
	if (g_chip == NULL)
		return -ENODEV;

	bank = g_chip->pin2id(g_chip, pin, &index);
	if (bank < 0 || index < 0 || bank >= g_chip->bank_size)
		return -EFAULT;

	pb = g_chip->banks + bank;
	if (g_chip->get_type)
		ret = g_chip->get_type(pb, index);
	return ret;
}
EXPORT_SYMBOL(mxs_get_type);

int mxs_set_type(unsigned int pin, enum pin_fun fun, const char *lab)
{
	int bank, index;
	struct pin_bank *pb;
	if (g_chip == NULL)
		return -ENODEV;

	bank = g_chip->pin2id(g_chip, pin, &index);
	if (bank < 0 || index < 0 || bank >= g_chip->bank_size)
		return -EFAULT;

	pb = g_chip->banks + bank;

	if (!test_bit(index, &pb->bitmap))
		return -ENOLCK;
	if (lab != pb->label[index])	/* label is const string */
		return -EINVAL;
	if (g_chip->set_type)
		g_chip->set_type(pb, index, fun);
	return 0;
}
EXPORT_SYMBOL(mxs_set_type);

int mxs_set_strength(unsigned int pin, enum pad_strength cfg, const char *lab)
{
	int bank, index;
	struct pin_bank *pb;
	if (g_chip == NULL)
		return -ENODEV;

	bank = g_chip->pin2id(g_chip, pin, &index);
	if (bank < 0 || index < 0 || bank >= g_chip->bank_size)
		return -EFAULT;

	pb = g_chip->banks + bank;

	if (!test_bit(index, &pb->bitmap))
		return -ENOLCK;
	if (lab != pb->label[index])	/* label is const string */
		return -EINVAL;
	if (g_chip->set_strength)
		g_chip->set_strength(pb, index, cfg);
	return 0;
}
EXPORT_SYMBOL(mxs_set_strength);

int mxs_set_voltage(unsigned int pin, enum pad_voltage cfg, const char *lab)
{
	int bank, index;
	struct pin_bank *pb;
	if (g_chip == NULL)
		return -ENODEV;

	bank = g_chip->pin2id(g_chip, pin, &index);
	if (bank < 0 || index < 0 || bank >= g_chip->bank_size)
		return -EFAULT;

	pb = g_chip->banks + bank;

	if (!test_bit(index, &pb->bitmap))
		return -ENOLCK;
	if (lab != pb->label[index])	/* label is const string */
		return -EINVAL;
	if (g_chip->set_voltage)
		g_chip->set_voltage(pb, index, cfg);
	return 0;
}
EXPORT_SYMBOL(mxs_set_voltage);

int mxs_set_pullup(unsigned int pin, int en, const char *lab)
{
	int bank, index;
	struct pin_bank *pb;
	if (g_chip == NULL)
		return -ENODEV;

	bank = g_chip->pin2id(g_chip, pin, &index);
	if (bank < 0 || index < 0 || bank >= g_chip->bank_size)
		return -EFAULT;

	pb = g_chip->banks + bank;

	if (!test_bit(index, &pb->bitmap))
		return -ENOLCK;
	if (lab != pb->label[index])	/* label is const string */
		return -EINVAL;
	if (g_chip->set_pullup)
		g_chip->set_pullup(pb, index, en);
	return 0;
}
EXPORT_SYMBOL(mxs_set_pullup);

void mxs_release_pin(unsigned int pin, const char *lab)
{
	int bank, index;
	struct pin_bank *pb;
	if (g_chip == NULL)
		return;

	bank = g_chip->pin2id(g_chip, pin, &index);
	if (bank < 0 || index < 0 || bank >= g_chip->bank_size)
		return;

	pb = g_chip->banks + bank;

	if (!test_bit(index, &pb->bitmap))
		return;
	if (lab != pb->label[index])	/* label is const string */
		return;
	pb->label[index] = NULL;

	clear_bit(index, &pb->bitmap);
}
EXPORT_SYMBOL(mxs_release_pin);

unsigned int mxs_pin2gpio(unsigned int pin)
{
	int bank, index;
	struct pin_bank *pb;
	if (g_chip == NULL)
		return -ENODEV;

	if (!g_chip->get_gpio)
		return -ENODEV;

	bank = g_chip->pin2id(g_chip, pin, &index);
	if (bank < 0 || index < 0 || bank >= g_chip->bank_size)
		return -EFAULT;

	pb = g_chip->banks + bank;

	return g_chip->get_gpio(pb, index);
}

int __init mxs_set_pinctrl_chip(struct pinctrl_chip *chip)
{
	if (!(chip && chip->banks && chip->bank_size &&
	      chip->get_gpio && chip->pin2id))
		return -EINVAL;

	if (g_chip)
		return -EEXIST;
	g_chip = chip;
	return 0;
};
