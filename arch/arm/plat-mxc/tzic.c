/*
 * Copyright (C) 2004-2010 Freescale Semiconductor, Inc. All Rights Reserved.
 *
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/errno.h>
#include <mach/hardware.h>
#include <linux/io.h>
#include <asm/irq.h>
#include <asm/mach/irq.h>
#include <mach/common.h>

/*
 *****************************************
 * TZIC Registers                        *
 *****************************************
 */
void __iomem *tzic_base;

#define TZIC_BASE		(tzic_base)
#define TZIC_INTCNTL            (TZIC_BASE + 0x0000)	/* control register */
#define TZIC_INTTYPE            (TZIC_BASE + 0x0004)	/* Controller type register */
#define TZIC_IMPID              (TZIC_BASE + 0x0008)	/* Distributor Implementer Identification Register */
#define TZIC_PRIOMASK           (TZIC_BASE + 0x000C)	/* Priority Mask Reg */
#define TZIC_SYNCCTRL           (TZIC_BASE + 0x0010)	/* Synchronizer Control register */
#define TZIC_DSMINT             (TZIC_BASE + 0x0014)	/* DSM interrupt Holdoffregister */
#define TZIC_INTSEC0            (TZIC_BASE + 0x0080)	/* interrupt security register 0 */
#define TZIC_ENSET0             (TZIC_BASE + 0x0100)	/* Enable Set Register 0 */
#define TZIC_ENCLEAR0		(TZIC_BASE + 0x0180)	/* Enable Clear Register 0 */
#define TZIC_SRCSET0            (TZIC_BASE + 0x0200)	/* Source Set Register 0 */
#define TZIC_SRCCLAR0           (TZIC_BASE + 0x0280)	/* Source Clear Register 0 */
#define TZIC_PRIORITY0          (TZIC_BASE + 0x0400)	/* Priority Register 0 */
#define TZIC_PND0               (TZIC_BASE + 0x0D00)	/* Pending Register 0 */
#define TZIC_HIPND0             (TZIC_BASE + 0x0D80)	/* High Priority Pending Register */
#define TZIC_WAKEUP0            (TZIC_BASE + 0x0E00)	/* Wakeup Config Register */
#define TZIC_SWINT              (TZIC_BASE + 0x0F00)	/* Software Interrupt Rigger Register */
#define TZIC_ID0                (TZIC_BASE + 0x0FD0)	/* Indentification Register 0 */

#define TZIC_NUM_IRQS		128

/**
 * tzic_mask_irq() - Disable interrupt number "irq" in the TZIC
 *
 * @param  irq          interrupt source number
 */
static void mxc_mask_irq(unsigned int irq)
{
	int index, off;

	index = irq >> 5;
	off = irq & 0x1F;
	__raw_writel(1 << off, TZIC_ENCLEAR0 + (index << 2));
}

/**
 * tzic_unmask_irq() - Enable interrupt number "irq" in the TZIC
 *
 * @param  irq          interrupt source number
 */
static void mxc_unmask_irq(unsigned int irq)
{
	int index, off;

	index = irq >> 5;
	off = irq & 0x1F;
	__raw_writel(1 << off, TZIC_ENSET0 + (index << 2));
}

static unsigned int wakeup_intr[4];

/**
 * tzic_set_wake_irq() - Set interrupt number "irq" in the TZIC as a wake-up source.
 *
 * @param  irq          interrupt source number
 * @param  enable       enable as wake-up if equal to non-zero
 * 			disble as wake-up if equal to zero
 *
 * @return       This function returns 0 on success.
 */
static int mxc_set_wake_irq(unsigned int irq, unsigned int enable)
{
	unsigned int index, off;

	index = irq >> 5;
	off = irq & 0x1F;

	if (index > 3)
		return -EINVAL;

	if (enable)
		wakeup_intr[index] |= (1 << off);
	else
		wakeup_intr[index] &= ~(1 << off);

	return 0;
}

static struct irq_chip mxc_tzic_chip = {
	.name = "MXC_TZIC",
	.ack = mxc_mask_irq,
	.mask = mxc_mask_irq,
	.unmask = mxc_unmask_irq,
	.set_wake = mxc_set_wake_irq,
};

/*!
 * This function initializes the TZIC hardware and disables all the
 * interrupts. It registers the interrupt enable and disable functions
 * to the kernel for each interrupt source.
 */
void __init mxc_tzic_init_irq(unsigned long base)
{
	int i;

	tzic_base = ioremap(base, SZ_4K);

	/* put the TZIC into the reset value with
	 * all interrupts disabled
	 */
	i = __raw_readl(TZIC_INTCNTL);

	__raw_writel(0x80010001, TZIC_INTCNTL);
	i = __raw_readl(TZIC_INTCNTL);
	__raw_writel(0x1f, TZIC_PRIOMASK);
	i = __raw_readl(TZIC_PRIOMASK);
	__raw_writel(0x02, TZIC_SYNCCTRL);
	i = __raw_readl(TZIC_SYNCCTRL);
	for (i = 0; i < 4; i++) {
		__raw_writel(0xFFFFFFFF, TZIC_INTSEC0 + i * 4);
	}
	/* disable all interrupts */
	for (i = 0; i < 4; i++) {
		__raw_writel(0xFFFFFFFF, TZIC_ENCLEAR0 + i * 4);
	}

	/* all IRQ no FIQ Warning :: No selection */

	for (i = 0; i < TZIC_NUM_IRQS; i++) {
		set_irq_chip(i, &mxc_tzic_chip);
		set_irq_handler(i, handle_level_irq);
		set_irq_flags(i, IRQF_VALID);
	}
	mxc_register_gpios();

	printk(KERN_INFO "MXC IRQ initialized\n");
}

/*!
 * enable wakeup interrupt
 *
 * @param is_idle		1 if called in idle loop (enset registers);
 *				0 to be used when called from low power entry
 * @return			0 if successful; non-zero otherwise
 */
int tzic_enable_wake(int is_idle)
{
	unsigned int i, v;

	__raw_writel(1, TZIC_DSMINT);
	if (unlikely(__raw_readl(TZIC_DSMINT) == 0))
		return -EAGAIN;

	if (likely(is_idle)) {
		for (i = 0; i < 4; i++) {
			v = __raw_readl(TZIC_ENSET0 + i * 4);
			__raw_writel(v, TZIC_WAKEUP0 + i * 4);
		}
	} else {
	for (i = 0; i < 4; i++) {
			v = wakeup_intr[i];
			__raw_writel(v, TZIC_WAKEUP0 + i * 4);
		}
	}
	return 0;
}
