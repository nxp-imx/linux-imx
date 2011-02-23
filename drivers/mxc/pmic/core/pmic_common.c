/*
 * Copyright 2009-2011 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

/*!
 * @file pmic_common.c
 * @brief This is the common file for the PMIC Core/Protocol driver.
 *
 * @ingroup PMIC_CORE
 */

/*
 * Includes
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/kthread.h>
#include <linux/pmic_external.h>
#include <linux/pmic_status.h>

#include <asm/uaccess.h>

#include "pmic.h"

/*
 * Global variables
 */
pmic_version_t mxc_pmic_version;
unsigned int active_events[MAX_ACTIVE_EVENTS];


static struct completion event_completion;
static struct task_struct *tstask;

static int pmic_event_thread_func(void *v)
{
	unsigned int loop;
	unsigned int count = 0;
	unsigned int irq = (int)v;

	while (1) {
		wait_for_completion_interruptible(
				&event_completion);
		if (kthread_should_stop())
			break;

		count = pmic_get_active_events(
				active_events);
		pr_debug("active events number %d\n", count);

	do {
		for (loop = 0; loop < count; loop++)
			pmic_event_callback(active_events[loop]);

		count = pmic_get_active_events(active_events);

	} while (count != 0);
		enable_irq(irq);
	}

	return 0;
}

int pmic_start_event_thread(int irq_num)
{
	int ret = 0;

	if (tstask)
		return ret;

	init_completion(&event_completion);

	tstask = kthread_run(pmic_event_thread_func,
		(void *)irq_num, "pmic-event-thread");
	ret = IS_ERR(tstask) ? -1 : 0;
	if (IS_ERR(tstask))
		tstask = NULL;
	return ret;
}

void pmic_stop_event_thread(void)
{
	if (tstask) {
		complete(&event_completion);
		kthread_stop(tstask);
	}
}

/*!
 * This function is called when pmic interrupt occurs on the processor.
 * It is the interrupt handler for the pmic module.
 *
 * @param        irq        the irq number
 * @param        dev_id     the pointer on the device
 *
 * @return       The function returns IRQ_HANDLED when handled.
 */
irqreturn_t pmic_irq_handler(int irq, void *dev_id)
{
	disable_irq_nosync(irq);
	complete(&event_completion);

	return IRQ_HANDLED;
}

/*!
 * This function is used to determine the PMIC type and its revision.
 *
 * @return      Returns the PMIC type and its revision.
 */

pmic_version_t pmic_get_version(void)
{
	return mxc_pmic_version;
}
EXPORT_SYMBOL(pmic_get_version);
