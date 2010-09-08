/*
 * Copyright 2008-2010 Freescale Semiconductor, Inc. All Rights Reserved.
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
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/mfd/mc9s08dz60/pmic.h>

#include <mach/hardware.h>
#include <asm/irq.h>
#include <asm/setup.h>
#include <asm/bitops.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/irq.h>

#include <mach/gpio.h>
#include <mach/irqs.h>
#include <mach/iomux-v3.h>
#include <mach/iomux-mx35.h>
#include "mach/board-mx35_3ds.h"

/*!
 * @file mach-mx35/mx35_3stack_irq.c
 *
 * @brief This file contains the board specific initialization routines.
 *
 * @ingroup MSL_MX35
 */

/*
 * The interrupt status and mask variables.
 */
static unsigned long pseudo_irq_pending;
static unsigned long pseudo_irq_enable;
static unsigned long pseudo_irq_wakeup;
static unsigned long pseudo_suspend;
static atomic_t pseudo_irq_state = ATOMIC_INIT(0);

/*
 * The declaration of handler of two work queue.
 * The one is the work queue to indentify the events from MCU.
 * The another is the work queue to change the events mask.
 */
static void mcu_event_handler(struct work_struct *work);
static void mcu_state_handler(struct work_struct *work);
static void mcu_event_delay(unsigned long data);

/*!
 * The work structure for mcu events.
 */
static DECLARE_WORK(mcu_event_ws, mcu_event_handler);
static DECLARE_WORK(mcu_state_ws, mcu_state_handler);
static DEFINE_TIMER(mcu_delay_timer, mcu_event_delay, HZ, 0);

static inline void mxc_pseudo_irq_ack(void)
{
	disable_irq(MXC_PSEUDO_PARENT);
	atomic_set(&pseudo_irq_state, 0);
}

static inline void mxc_pseudo_irq_trigger(void)
{
	if (!atomic_xchg(&pseudo_irq_state, 1))
		enable_irq(MXC_PSEUDO_PARENT);
}

/*
 * mask a pseudo interrupt by setting the bit in the mask variable.
 * @param irq           a pseudo virtual irq number
 */
static void pseudo_mask_irq(u32 irq)
{
	int index = irq - MXC_PSEUDO_IO_BASE;
	clear_bit(index, &pseudo_irq_enable);
}

/*
 * disable a pseudo interrupt by triggerring a work queue
 * @param irq           a pseudo virtual irq number
 */
static void pseudo_disable_irq(u32 irq)
{
	struct irq_desc *desc = irq_desc + irq;
	desc->chip->mask(irq);
	desc->status |= IRQ_MASKED;
	schedule_work(&mcu_state_ws);
}

/*
 * Acknowledge a pseudo interrupt by clearing the bit in the isr variable.
 * @param irq           a pseudo virtual irq number
 */
static void pseudo_ack_irq(u32 irq)
{
	int index = irq - MXC_PSEUDO_IO_BASE;
	/* clear the interrupt status */
	clear_bit(index, &pseudo_irq_pending);
}

/*
 * unmask a pseudo interrupt by clearing the bit in the imr.
 * @param irq           a pseudo virtual irq number
 */
static void pseudo_unmask_irq(u32 irq)
{
	int index = irq - MXC_PSEUDO_IO_BASE;

	set_bit(index, &pseudo_irq_enable);

	if (test_bit(index, &pseudo_irq_pending))
		mxc_pseudo_irq_trigger();
}

/*
 * Enable a pseudo interrupt by triggerring a work queue
 * @param irq           a pseudo virtual irq number
 */
static void pseudo_enable_irq(u32 irq)
{
	struct irq_desc *desc = irq_desc + irq;
	desc->chip->unmask(irq);
	desc->status &= ~IRQ_MASKED;
	schedule_work(&mcu_state_ws);
}

/*
 * set pseudo irq as a wake-up source.
 * @param irq           a pseudo virtual irq number
 * @param enable	enable as wake-up if equal to non-ero
 * @return 	This function return 0 on success
 */
static int pseudo_set_wake_irq(u32 irq, u32 enable)
{
	int index = irq - MXC_PSEUDO_IO_BASE;

	if (index >= 16)
		return -ENODEV;

	if (enable) {
		if (!pseudo_irq_wakeup)
			enable_irq_wake(gpio_to_irq(0));
		pseudo_irq_wakeup |= (1 << index);
	} else {
		pseudo_irq_wakeup &= ~(1 << index);
		if (!pseudo_irq_wakeup)
			disable_irq_wake(gpio_to_irq(0));
	}
	return 0;
}

static struct irq_chip pseudo_irq_chip = {
	.ack = pseudo_ack_irq,
	.mask = pseudo_mask_irq,
	.disable = pseudo_disable_irq,
	.unmask = pseudo_unmask_irq,
	.enable = pseudo_enable_irq,
	.set_wake = pseudo_set_wake_irq,
};

static void mxc_pseudo_irq_handler(u32 irq, struct irq_desc *desc)
{
	u32 pseudo_irq;
	u32 index, mask;

	desc->chip->mask(irq);
	mxc_pseudo_irq_ack();

	mask = pseudo_irq_enable;
	index = pseudo_irq_pending;

	if (unlikely(!(index & mask))) {
		printk(KERN_ERR "\nPseudo IRQ: Spurious interrupt:0x%0x\n\n",
		       index);
		pr_info("IEN=0x%x, PENDING=0x%x\n", mask, index);
		return;
	}

	index = index & mask;
	pseudo_irq = MXC_PSEUDO_IO_BASE;
	for (; index != 0; index >>= 1, pseudo_irq++) {
		struct irq_desc *d;
		if ((index & 1) == 0)
			continue;
		d = irq_desc + pseudo_irq;
		if (unlikely(!(d->handle_irq))) {
			printk(KERN_ERR "\nPseudo irq: %d unhandeled\n",
			       pseudo_irq);
			BUG();	/* oops */
		}
		d->handle_irq(pseudo_irq, d);
		d->chip->ack(pseudo_irq);
	}
}

static void mcu_event_delay(unsigned long data)
{
	schedule_work(&mcu_event_ws);
}

/*!
 * This function is called when mcu interrupt occurs on the processor.
 * It is the interrupt handler for the mcu.
 *
 * @param        irq        the irq number
 * @param        dev_id     the pointer on the device
 *
 * @return       The function returns IRQ_HANDLED when handled.
 */
static irqreturn_t mcu_irq_handler(int irq, void *dev_id)
{
	disable_irq_nosync(gpio_to_irq(0));
	if (pseudo_suspend)
		mod_timer(&mcu_delay_timer, jiffies + HZ);
	else
		schedule_work(&mcu_event_ws);

	return IRQ_HANDLED;
}

/*!
 * This function is the work handler of mcu interrupt.
 * It reads the events status and trigger the pseudo irq.
 */
static void mcu_event_handler(struct work_struct *work)
{
	int i, err;
	unsigned int flag1, flag2;

	/* read int flags and ack int */
	for (i = 0; i < 3; i++) {
		err = mcu_pmic_read_reg(REG_MCU_INT_FLAG_1, &flag1, 0xFFFFFFFF);
		err |= mcu_pmic_read_reg(REG_MCU_INT_FLAG_2,
			&flag2, 0xFFFFFFFF);
		err |= mcu_pmic_write_reg(REG_MCU_INT_FLAG_1, 0, 0xFFFFFFFF);
		err |= mcu_pmic_write_reg(REG_MCU_INT_FLAG_2, 0, 0xFFFFFFFF);
		if (err == 0)
			break;
	}

	if (i >= 3) {
		printk(KERN_ERR "Reads MCU event fail\n");
		goto no_new_events;
	}

	for (i = 0; flag1 && (i < MCU_INT_RTC); i++, flag1 >>= 1)
		if (flag1 & 1)
			set_bit(i, &pseudo_irq_pending);

	for (i = MCU_INT_RTC; flag2 && (i <= MCU_INT_KEYPAD); i++, flag2 >>= 1)
		if (flag2 & 1)
			set_bit(i, &pseudo_irq_pending);
no_new_events:
	if (pseudo_irq_pending & pseudo_irq_enable)
		mxc_pseudo_irq_trigger();
	enable_irq(gpio_to_irq(0));
}

static void mcu_state_handler(struct work_struct *work)
{
	int err, i;
	unsigned int event1, event2;
	event1 = pseudo_irq_enable & ((1 << MCU_INT_RTC) - 1);
	event2 = pseudo_irq_enable >> MCU_INT_RTC;

	if (is_suspend_ops_started())
		return;

	for (i = 0; i < 3; i++) {
		err = mcu_pmic_write_reg(REG_MCU_INT_ENABLE_1, event1, 0xFF);
		err |= mcu_pmic_write_reg(REG_MCU_INT_ENABLE_2, event2, 0xFF);
		if (err == 0)
			break;
	}
	if (i >= 3)
		printk(KERN_ERR "Change MCU event mask fail\n");
}

static int __init mxc_pseudo_init(void)
{
	int i;

	/* disable the interrupt and clear the status */
	pseudo_irq_pending = 0;
	pseudo_irq_enable = 0;

	pr_info("3-Stack Pseudo interrupt rev=0.1v\n");

	for (i = MXC_PSEUDO_IO_BASE;
	     i < (MXC_PSEUDO_IO_BASE + 16); i++) {
		set_irq_chip(i, &pseudo_irq_chip);
		set_irq_handler(i, handle_simple_irq);
		set_irq_flags(i, IRQF_VALID);
	}

	set_irq_flags(MXC_PSEUDO_PARENT, IRQF_NOAUTOEN);
	set_irq_handler(MXC_PSEUDO_PARENT, mxc_pseudo_irq_handler);

	/* Set and install PMIC IRQ handler */
	gpio_request(0, NULL);
	gpio_direction_input(0);

	set_irq_type(gpio_to_irq(0), IRQF_TRIGGER_RISING);
	if (request_irq(gpio_to_irq(0), mcu_irq_handler,
			0, "MCU_IRQ", 0)) {
		printk(KERN_ERR "mcu request irq failed\n");
		return -1;
	}
	return 0;
}

fs_initcall_sync(mxc_pseudo_init);

static int mxc_pseudo_irq_suspend(struct platform_device *dev,
				  pm_message_t mesg)
{
	int err, i;
	unsigned int event1, event2;

	if (!pseudo_irq_wakeup)
		return 0;

	event1 = pseudo_irq_wakeup & ((1 << MCU_INT_RTC) - 1);
	event2 = pseudo_irq_wakeup >> MCU_INT_RTC;

	for (i = 0; i < 3; i++) {
		err = mcu_pmic_write_reg(REG_MCU_INT_ENABLE_1, event1, 0xFF);
		err |= mcu_pmic_write_reg(REG_MCU_INT_ENABLE_2, event2, 0xFF);
		if (err == 0)
			break;
	}
	pseudo_suspend = 1;
	return err;
}

static int mxc_pseudo_irq_resume(struct platform_device *dev)
{
	if (!pseudo_irq_wakeup)
		return 0;

	schedule_work(&mcu_state_ws);
	pseudo_suspend = 0;
	return 0;
}

static struct platform_driver mxc_pseudo_irq_driver = {
	.driver = {
		   .name = "mxc_pseudo_irq",
		   },
	.suspend = mxc_pseudo_irq_suspend,
	.resume = mxc_pseudo_irq_resume,
};

static int __init mxc_pseudo_sysinit(void)
{
	return platform_driver_register(&mxc_pseudo_irq_driver);
}

late_initcall(mxc_pseudo_sysinit);
