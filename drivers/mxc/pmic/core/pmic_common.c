/*
 * Copyright 2009 Freescale Semiconductor, Inc. All Rights Reserved.
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
#include <linux/pmic_external.h>
#include <linux/pmic_status.h>

#include <asm/uaccess.h>

#include "pmic.h"

/*
 * Global variables
 */
pmic_version_t mxc_pmic_version;
unsigned int active_events[MAX_ACTIVE_EVENTS];
struct workqueue_struct *pmic_event_wq;

void pmic_bh_handler(struct work_struct *work);
/*!
 * Bottom half handler of PMIC event handling.
 */
DECLARE_WORK(pmic_ws, pmic_bh_handler);

/*!
 * This function is the bottom half handler of the PMIC interrupt.
 * It checks for active events and launches callback for the
 * active events.
 */
void pmic_bh_handler(struct work_struct *work)
{
	unsigned int loop;
	unsigned int count = 0;

	count = pmic_get_active_events(active_events);
	pr_debug("active events number %d\n", count);

	for (loop = 0; loop < count; loop++)
		pmic_event_callback(active_events[loop]);

	return;
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
	/* prepare a task */
	queue_work(pmic_event_wq, &pmic_ws);

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
