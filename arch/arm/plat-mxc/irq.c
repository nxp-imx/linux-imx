/*
 * Copyright 2004-2010 Freescale Semiconductor, Inc. All Rights Reserved.
 * Copyright 2008 Juergen Beisert, kernel@pengutronix.de
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
 * MA  02110-1301, USA.
 */

#include <linux/module.h>
#include <linux/irq.h>
#include <linux/io.h>
#include <linux/sysdev.h>
#include <mach/common.h>
#include <asm/mach/irq.h>
#include <mach/hardware.h>

#define AVIC_INTCNTL		0x00	/* int control reg */
#define AVIC_NIMASK		0x04	/* int mask reg */
#define AVIC_INTENNUM		0x08	/* int enable number reg */
#define AVIC_INTDISNUM		0x0C	/* int disable number reg */
#define AVIC_INTENABLEH		0x10	/* int enable reg high */
#define AVIC_INTENABLEL		0x14	/* int enable reg low */
#define AVIC_INTTYPEH		0x18	/* int type reg high */
#define AVIC_INTTYPEL		0x1C	/* int type reg low */
#define AVIC_NIPRIORITY(x)	(0x20 + 4 * (7 - (x))) /* int priority */
#define AVIC_NIVECSR		0x40	/* norm int vector/status */
#define AVIC_FIVECSR		0x44	/* fast int vector/status */
#define AVIC_INTSRCH		0x48	/* int source reg high */
#define AVIC_INTSRCL		0x4C	/* int source reg low */
#define AVIC_INTFRCH		0x50	/* int force reg high */
#define AVIC_INTFRCL		0x54	/* int force reg low */
#define AVIC_NIPNDH		0x58	/* norm int pending high */
#define AVIC_NIPNDL		0x5C	/* norm int pending low */
#define AVIC_FIPNDH		0x60	/* fast int pending high */
#define AVIC_FIPNDL		0x64	/* fast int pending low */

void __iomem *avic_base;

#define IRQ_BIT(irq)  (1 << (irq))

static uint32_t saved_wakeup_low, saved_wakeup_high;
static uint32_t suspend_wakeup_low, suspend_wakeup_high;

int imx_irq_set_priority(unsigned char irq, unsigned char prio)
{
#ifdef CONFIG_MXC_IRQ_PRIOR
	unsigned int temp;
	unsigned int mask = 0x0F << irq % 8 * 4;

	if (irq >= MXC_INTERNAL_IRQS)
		return -EINVAL;;

	temp = __raw_readl(avic_base + AVIC_NIPRIORITY(irq / 8));
	temp &= ~mask;
	temp |= prio & mask;

	__raw_writel(temp, avic_base + AVIC_NIPRIORITY(irq / 8));

	return 0;
#else
	return -ENOSYS;
#endif
}
EXPORT_SYMBOL(imx_irq_set_priority);

#ifdef CONFIG_FIQ
int mxc_set_irq_fiq(unsigned int irq, unsigned int type)
{
	unsigned int irqt;

	if (irq >= MXC_INTERNAL_IRQS)
		return -EINVAL;

	if (irq < MXC_INTERNAL_IRQS / 2) {
		irqt = __raw_readl(avic_base + AVIC_INTTYPEL) & ~(1 << irq);
		__raw_writel(irqt | (!!type << irq), avic_base + AVIC_INTTYPEL);
	} else {
		irq -= MXC_INTERNAL_IRQS / 2;
		irqt = __raw_readl(avic_base + AVIC_INTTYPEH) & ~(1 << irq);
		__raw_writel(irqt | (!!type << irq), avic_base + AVIC_INTTYPEH);
	}

	return 0;
}
EXPORT_SYMBOL(mxc_set_irq_fiq);
#endif /* CONFIG_FIQ */

/* Disable interrupt number "irq" in the AVIC */
static void mxc_mask_irq(unsigned int irq)
{
	__raw_writel(irq, avic_base + AVIC_INTDISNUM);
}

/* Enable interrupt number "irq" in the AVIC */
static void mxc_unmask_irq(unsigned int irq)
{
	__raw_writel(irq, avic_base + AVIC_INTENNUM);
}

/*!
 * Set interrupt number "irq" in the AVIC as a wake-up source.
 *
 * @param  irq          interrupt source number
 * @param  enable       enable as wake-up if equal to non-zero
 * 			disble as wake-up if equal to zero
 *
 * @return       This function returns 0 on success.
 */
static int mxc_set_wake_irq(unsigned int irq, unsigned int enable)
{
	uint32_t *wakeup_intr;
	uint32_t irq_bit;

	if (irq < 32) {
		wakeup_intr = &suspend_wakeup_low;
		irq_bit = IRQ_BIT(irq);
	} else {
		wakeup_intr = &suspend_wakeup_high;
		irq_bit = IRQ_BIT(irq - 32);
	}

	if (enable) {
		*wakeup_intr |= irq_bit;
	} else {
		*wakeup_intr &= ~irq_bit;
	}

	return 0;
}

static struct irq_chip mxc_avic_chip = {
	.ack = mxc_mask_irq,
	.mask = mxc_mask_irq,
	.unmask = mxc_unmask_irq,
	.set_wake = mxc_set_wake_irq,
};

#ifdef CONFIG_PM
/*!
 * This function puts the AVIC in low-power mode/state.
 * All the interrupts that are enabled are first saved.
 * Only those interrupts which registers as a wake source by calling
 * enable_irq_wake are enabled. All other interrupts are disabled.
 *
 * @param   dev  the system device structure used to give information
 *                on AVIC to suspend
 * @param   mesg the power state the device is entering
 *
 * @return  The function always returns 0.
 */
static int mxc_avic_suspend(struct sys_device *dev, pm_message_t mesg)
{
	saved_wakeup_high = __raw_readl(avic_base + AVIC_INTENABLEH);
	saved_wakeup_low = __raw_readl(avic_base + AVIC_INTENABLEL);

	__raw_writel(suspend_wakeup_high, avic_base + AVIC_INTENABLEH);
	__raw_writel(suspend_wakeup_low, avic_base + AVIC_INTENABLEL);

	return 0;
}

/*!
 * This function brings the AVIC back from low-power state.
 * All the interrupts enabled before suspension are re-enabled from
 * the saved information.
 *
 * @param   dev  the system device structure used to give information
 *                on AVIC to resume
 *
 * @return  The function always returns 0.
 */
static int mxc_avic_resume(struct sys_device *dev)
{
	__raw_writel(saved_wakeup_high, avic_base + AVIC_INTENABLEH);
	__raw_writel(saved_wakeup_low, avic_base + AVIC_INTENABLEL);

	return 0;
}

#else
#define mxc_avic_suspend  NULL
#define mxc_avic_resume   NULL
#endif				/* CONFIG_PM */
/*!
 * This structure contains pointers to the power management callback functions.
 */
static struct sysdev_class mxc_avic_sysclass = {
	.name = "mxc_irq",
	.suspend = mxc_avic_suspend,
	.resume = mxc_avic_resume,
};

/*!
 * This structure represents AVIC as a system device.
 * System devices follow a slightly different driver model.
 * They don't need to do dynammic driver binding, can't be probed,
 * and don't reside on any type of peripheral bus.
 * So, it is represented and treated a little differently.
 */
static struct sys_device mxc_avic_device = {
	.id = 0,
	.cls = &mxc_avic_sysclass,
};

/*
 * This function is used to get the AVIC Lo and Hi interrupts
 * that are enabled as wake up sources to wake up the core from suspend
 */
void mxc_get_wake_irq(u32 *wake_src[])
{
	*wake_src[0] = __raw_readl(avic_base + AVIC_INTENABLEL);
	*wake_src[1] = __raw_readl(avic_base + AVIC_INTENABLEH);
}

/*
 * This function initializes the AVIC hardware and disables all the
 * interrupts. It registers the interrupt enable and disable functions
 * to the kernel for each interrupt source.
 */
void __init mxc_init_irq(void __iomem *irqbase)
{
	int i;

	avic_base = irqbase;

	/* put the AVIC into the reset value with
	 * all interrupts disabled
	 */
	__raw_writel(0, avic_base + AVIC_INTCNTL);
	__raw_writel(0x1f, avic_base + AVIC_NIMASK);

	/* disable all interrupts */
	__raw_writel(0, avic_base + AVIC_INTENABLEH);
	__raw_writel(0, avic_base + AVIC_INTENABLEL);

	/* all IRQ no FIQ */
	__raw_writel(0, avic_base + AVIC_INTTYPEH);
	__raw_writel(0, avic_base + AVIC_INTTYPEL);
	for (i = 0; i < MXC_INTERNAL_IRQS; i++) {
		set_irq_chip(i, &mxc_avic_chip);
		set_irq_handler(i, handle_level_irq);
		set_irq_flags(i, IRQF_VALID);
	}

	/* Set default priority value (0) for all IRQ's */
	for (i = 0; i < 8; i++)
		__raw_writel(0, avic_base + AVIC_NIPRIORITY(i));

	/* init architectures chained interrupt handler */
	mxc_register_gpios();

#ifdef CONFIG_FIQ
	/* Initialize FIQ */
	init_FIQ();
#endif

	if (MXC_INT_FORCE >= 32)
		__raw_writel(1 << (MXC_INT_FORCE & 31), avic_base + AVIC_INTFRCH);
	else if (MXC_INT_FORCE >= 0)
		__raw_writel(1 << MXC_INT_FORCE, avic_base + AVIC_INTFRCL);

	printk(KERN_INFO "MXC IRQ initialized\n");
}

/*!
 * This function registers AVIC hardware as a system device.
 * System devices will only be suspended with interrupts disabled, and
 * after all other devices have been suspended. On resume, they will be
 * resumed before any other devices, and also with interrupts disabled.
 *
 * @return       This function returns 0 on success.
 */
static int __init mxc_avic_sysinit(void)
{
	int ret = 0;

	ret = sysdev_class_register(&mxc_avic_sysclass);
	if (ret == 0) {
		ret = sysdev_register(&mxc_avic_device);
	}

	return ret;
}

arch_initcall(mxc_avic_sysinit);

