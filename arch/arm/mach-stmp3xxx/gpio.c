/*
 * Freescale STMP37XX/STMP378X GPIO driver
 *
 * Author: dmitry pervushin <dimka@embeddedalley.com>
 *
 * Copyright 2008-2009 Freescale Semiconductor, Inc. All Rights Reserved.
 * Copyright 2008 Embedded Alley Solutions, Inc All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/sysdev.h>
#include <linux/bitops.h>
#include <linux/irq.h>
#include <mach/hardware.h>
#include <mach/stmp3xxx_regs.h>

#include "common.h"

#define STMP3xxx_GPIO_TOTAL (3*32)	/* three banks by 32 pins each */

int gpio_request(unsigned id, char *label)
{
	BUG_ON(id > STMP3xxx_GPIO_TOTAL);
	return stmp3xxx_request_pin(id, PIN_GPIO, label);
}
EXPORT_SYMBOL(gpio_request);

void gpio_free(unsigned id)
{
	BUG_ON(id > STMP3xxx_GPIO_TOTAL);
	stmp3xxx_release_pin(id, NULL);
}
EXPORT_SYMBOL(gpio_free);

void gpio_set_value(unsigned id, int value)
{
	struct stmp3xxx_pinmux_bank *b;
	int num = STMP3XXX_PINID_TO_PINNUM(id);

	BUG_ON(id > STMP3xxx_GPIO_TOTAL);
	b = stmp_pinmux_banks(STMP3XXX_PINID_TO_BANK(id));
	spin_lock(&b->lock);
	__raw_writel(1 << num, value ? b->hw_gpio_set : b->hw_gpio_clr);
	spin_unlock(&b->lock);
}
EXPORT_SYMBOL(gpio_set_value);

int gpio_get_value(unsigned id)
{
	struct stmp3xxx_pinmux_bank *b;
	int v;
	int num = STMP3XXX_PINID_TO_PINNUM(id);

	BUG_ON(id > STMP3xxx_GPIO_TOTAL);
	b = stmp_pinmux_banks(STMP3XXX_PINID_TO_BANK(id));
	spin_lock(&b->lock);
	v = (__raw_readl(b->hw_gpio_read) & (1 << num)) >> num;
	spin_unlock(&b->lock);
	return v;
}
EXPORT_SYMBOL(gpio_get_value);

int gpio_to_irq(unsigned id)
{
	struct stmp3xxx_pinmux_bank *b;
	BUG_ON(id > STMP3xxx_GPIO_TOTAL);
	b = stmp_pinmux_banks(STMP3XXX_PINID_TO_BANK(id));
	return b->irq;
}
EXPORT_SYMBOL(gpio_to_irq);

void gpio_direction_output(unsigned id, int value)
{
	struct stmp3xxx_pinmux_bank *b;
	int num = STMP3XXX_PINID_TO_PINNUM(id);

	BUG_ON(id > STMP3xxx_GPIO_TOTAL);
	b = stmp_pinmux_banks(STMP3XXX_PINID_TO_BANK(id));
	spin_lock(&b->lock);
	__raw_writel(1 << num, b->hw_gpio_doe + HW_STMP3xxx_SET);
	__raw_writel(1 << num, value ? b->hw_gpio_set : b->hw_gpio_clr);
	spin_unlock(&b->lock);
}
EXPORT_SYMBOL(gpio_direction_output);

void gpio_direction_input(unsigned id)
{
	struct stmp3xxx_pinmux_bank *b;
	int num = STMP3XXX_PINID_TO_PINNUM(id);

	BUG_ON(id > STMP3xxx_GPIO_TOTAL);
	b = stmp_pinmux_banks(STMP3XXX_PINID_TO_BANK(id));
	spin_lock(&b->lock);
	__raw_writel(1 << num, b->hw_gpio_doe + HW_STMP3xxx_CLR);
	spin_unlock(&b->lock);
}
EXPORT_SYMBOL(gpio_direction_input);

MODULE_AUTHOR("dmitry pervushin");
MODULE_LICENSE("GPL");
