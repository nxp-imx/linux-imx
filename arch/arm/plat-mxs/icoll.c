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
#include <linux/delay.h>

#include <mach/hardware.h>
#include <mach/device.h>

#include "regs-icoll.h"

void __iomem *g_icoll_base;

/*
 * IRQ handling
 */
static void icoll_ack_irq(unsigned int irq)
{
	__raw_writel(0, g_icoll_base + HW_ICOLL_VECTOR);

	/* ACK current interrupt */
	__raw_writel(BV_ICOLL_LEVELACK_IRQLEVELACK__LEVEL0,
		     g_icoll_base + HW_ICOLL_LEVELACK);

	/* Barrier */
	(void)__raw_readl(g_icoll_base + HW_ICOLL_STAT);
}

static void icoll_mask_irq(unsigned int irq)
{
	__raw_writel(BM_ICOLL_INTERRUPTn_ENABLE,
		     g_icoll_base + HW_ICOLL_INTERRUPTn_CLR(irq));
}

static void icoll_unmask_irq(unsigned int irq)
{
	__raw_writel(BM_ICOLL_INTERRUPTn_ENABLE,
		     g_icoll_base + HW_ICOLL_INTERRUPTn_SET(irq));
}

static struct irq_chip icoll_chip = {
	.ack = icoll_ack_irq,
	.mask = icoll_mask_irq,
	.unmask = icoll_unmask_irq,
};

void __init avic_init_irq(void __iomem *base, int nr_irqs)
{
	int i;
	g_icoll_base = base;

	/* Reset icoll */
	__raw_writel(BM_ICOLL_CTRL_SFTRST, g_icoll_base + HW_ICOLL_CTRL_CLR);

	for (i = 0; i < 100000; i++) {
		if (!(__raw_readl(g_icoll_base + HW_ICOLL_CTRL) &
		      BM_ICOLL_CTRL_SFTRST))
			break;
		udelay(2);
	}
	if (i >= 100000) {
		printk(KERN_ERR "%s:%d timeout when enableing\n",
		       __func__, __LINE__);
		return;
	}
	__raw_writel(BM_ICOLL_CTRL_CLKGATE, g_icoll_base + HW_ICOLL_CTRL_CLR);

	for (i = 0; i < nr_irqs; i++) {
		__raw_writel(0, g_icoll_base + HW_ICOLL_INTERRUPTn(i));
		set_irq_chip(i, &icoll_chip);
		set_irq_handler(i, handle_level_irq);
		set_irq_flags(i, IRQF_VALID | IRQF_PROBE);
	}

	__raw_writel(BF_ICOLL_LEVELACK_IRQLEVELACK
		     (BV_ICOLL_LEVELACK_IRQLEVELACK__LEVEL0),
		     g_icoll_base + HW_ICOLL_LEVELACK);
	__raw_writel(BF_ICOLL_LEVELACK_IRQLEVELACK
		     (BV_ICOLL_LEVELACK_IRQLEVELACK__LEVEL1),
		     g_icoll_base + HW_ICOLL_LEVELACK);
	__raw_writel(BF_ICOLL_LEVELACK_IRQLEVELACK
		     (BV_ICOLL_LEVELACK_IRQLEVELACK__LEVEL2),
		     g_icoll_base + HW_ICOLL_LEVELACK);
	__raw_writel(BF_ICOLL_LEVELACK_IRQLEVELACK
		     (BV_ICOLL_LEVELACK_IRQLEVELACK__LEVEL3),
		     g_icoll_base + HW_ICOLL_LEVELACK);

	__raw_writel(0, g_icoll_base + HW_ICOLL_VECTOR);
	/* Barrier */
	(void)__raw_readl(g_icoll_base + HW_ICOLL_STAT);
}
