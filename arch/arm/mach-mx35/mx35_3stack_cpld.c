/*
 * Copyright 2008-2009 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

#include <linux/types.h>
#include <linux/sched.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/clk.h>

#include <asm/irq.h>
#include <asm/setup.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <mach/hardware.h>
#include <mach/gpio.h>

#include "board-mx35_3stack.h"
#include "crm_regs.h"
#include "iomux.h"

/*!
 * @file mach-mx35/mx35_3stack_cpld.c
 *
 * @brief This file contains the board specific initialization routines.
 *
 * @ingroup MSL_MX35
 */
static void mxc_expio_irq_handler(u32 irq, struct irq_desc *desc)
{
	u32 expio_irq;
	u32 index, mask;
	desc->chip->mask(irq);	/* irq = gpio irq number */

	index = __raw_readw(mx35_3stack_board_io + INTR_STATUS_REG);
	mask = __raw_readw(mx35_3stack_board_io + INTR_MASK_REG);

	if (unlikely(!(index & (~mask)))) {
		printk(KERN_ERR "\nEXPIO: Spurious interrupt:0x%0x\n\n", index);
		pr_info("CPLD IMR(0x38)=0x%x, PENDING(0x28)=0x%x\n", mask,
			index);
		goto out;
	}
	index = index & (~mask);
	expio_irq = MXC_BOARD_IRQ_START;
	for (; index != 0; index >>= 1, expio_irq++) {
		struct irq_desc *d;
		if ((index & 1) == 0)
			continue;
		d = irq_desc + expio_irq;
		if (unlikely(!(d->handle_irq))) {
			printk(KERN_ERR "\nEXPIO irq: %d unhandeled\n",
			       expio_irq);
			BUG();	/* oops */
		}
		d->handle_irq(expio_irq, d);
	}

      out:
	desc->chip->ack(irq);
	desc->chip->unmask(irq);
}

/*
 * Disable an expio pin's interrupt by setting the bit in the imr.
 * @param irq           an expio virtual irq number
 */
static void expio_mask_irq(u32 irq)
{
	u16 reg, expio = MXC_IRQ_TO_EXPIO(irq);

	reg = __raw_readw(mx35_3stack_board_io + INTR_MASK_REG);
	/* mask the interrupt */
	__raw_writew(reg | (1 << expio), mx35_3stack_board_io + INTR_MASK_REG);
}

/*
 * Acknowledge an expanded io pin's interrupt by clearing the bit in the isr.
 * @param irq           an expanded io virtual irq number
 */
static void expio_ack_irq(u32 irq)
{
	u32 expio = MXC_IRQ_TO_EXPIO(irq);
	/* clear the interrupt status */
	__raw_writew(1 << expio, mx35_3stack_board_io + INTR_RESET_REG);
	__raw_writew(0, mx35_3stack_board_io + INTR_RESET_REG);
	/* mask the interrupt */
	expio_mask_irq(irq);
}

/*
 * Enable a expio pin's interrupt by clearing the bit in the imr.
 * @param irq           a expio virtual irq number
 */
static void expio_unmask_irq(u32 irq)
{
	u16 reg, expio = MXC_IRQ_TO_EXPIO(irq);

	reg = __raw_readw(mx35_3stack_board_io + INTR_MASK_REG);
	/* unmask the interrupt */
	__raw_writew(reg & (~(1 << expio)),
		     mx35_3stack_board_io + INTR_MASK_REG);
}

static struct irq_chip expio_irq_chip = {
	.ack = expio_ack_irq,
	.mask = expio_mask_irq,
	.unmask = expio_unmask_irq,
};

static int __init mxc_expio_init(void)
{
	int i;

	mx35_3stack_board_io = (u32) ioremap(BOARD_IO_ADDR, SZ_4K);
	if (mx35_3stack_board_io == 0)
		return -ENOMEM;

	if ((__raw_readw(mx35_3stack_board_io + MAGIC_NUMBER1_REG) != 0xAAAA) ||
	    (__raw_readw(mx35_3stack_board_io + MAGIC_NUMBER2_REG) != 0x5555))
		return -ENODEV;

	pr_info("3-Stack Debug board detected, rev = 0x%04X\n",
		readw(mx35_3stack_board_io + CPLD_CODE_VER_REG));

	/*
	 * Configure INT line as GPIO input
	 */
	mxc_request_iomux(EXPIO_PARENT_INT, MUX_CONFIG_FUNC);
	gpio_request(IOMUX_TO_GPIO(EXPIO_PARENT_INT), NULL);
	gpio_direction_input(IOMUX_TO_GPIO(EXPIO_PARENT_INT));

	/* disable the interrupt and clear the status */
	__raw_writew(0, mx35_3stack_board_io + INTR_MASK_REG);
	__raw_writew(0xFFFF, mx35_3stack_board_io + INTR_RESET_REG);
	__raw_writew(0, mx35_3stack_board_io + INTR_RESET_REG);
	__raw_writew(0x1F, mx35_3stack_board_io + INTR_MASK_REG);
	for (i = MXC_BOARD_IRQ_START; i < (MXC_BOARD_IRQ_START + MXC_BOARD_IRQS);
	     i++) {
		set_irq_chip(i, &expio_irq_chip);
		set_irq_handler(i, handle_level_irq);
		set_irq_flags(i, IRQF_VALID);
	}
	set_irq_type(IOMUX_TO_IRQ(EXPIO_PARENT_INT), IRQF_TRIGGER_LOW);
	set_irq_chained_handler(IOMUX_TO_IRQ(EXPIO_PARENT_INT),
				mxc_expio_irq_handler);
	return 0;
}

arch_initcall(mxc_expio_init);
