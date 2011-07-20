/*
 * Copyright (C) 2011 Freescale Semiconductor, Inc. All Rights Reserved.
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
#include <linux/platform_device.h>
#include <linux/irq.h>
#include <asm/hardware/gic.h>
#include <mach/hardware.h>

int mx6q_register_gpios(void);
unsigned int gpc_wake_irq[4];

static int mx6_gic_irq_set_wake(struct irq_data *d, unsigned int enable)
{
	if ((d->irq < MXC_INT_START) || (d->irq > MXC_INT_END)) {
		printk(KERN_ERR "Invalid irq number!\n");
		return -EINVAL;
	}

	if (enable) {
		gpc_wake_irq[d->irq / 32 - 1] |= 1 << (d->irq % 32);
		printk(KERN_INFO "add wake up source irq %d\n", d->irq);
	} else {
		printk(KERN_INFO "remove wake up source irq %d\n", d->irq);
		gpc_wake_irq[d->irq / 32 - 1] &= ~(1 << (d->irq % 32));
	}
	return 0;
}
void __init mx6_init_irq(void)
{
	struct irq_desc *desc;
	unsigned int i;

	gic_init(0, 29, IO_ADDRESS(IC_DISTRIBUTOR_BASE_ADDR),
		IO_ADDRESS(IC_INTERFACES_BASE_ADDR));

	for (i = MXC_INT_START; i <= MXC_INT_END; i++) {
		desc = irq_to_desc(i);
		desc->irq_data.chip->irq_set_wake = mx6_gic_irq_set_wake;
	}
	mx6q_register_gpios();
}
