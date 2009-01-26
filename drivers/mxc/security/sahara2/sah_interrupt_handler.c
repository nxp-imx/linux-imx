/*
 * Copyright 2004-2008 Freescale Semiconductor, Inc. All Rights Reserved.
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
* @file sah_interrupt_handler.c
*
* @brief Provides a hardware interrupt handling mechanism for device driver.
*
* This file needs to be ported for a non-Linux OS.
*
* It gets a call at #sah_Intr_Init() during initialization.
*
* #sah_Intr_Top_Half() is intended to be the Interrupt Service Routine.  It
* calls a portable function in another file to process the Sahara status.
*
* #sah_Intr_Bottom_Half() is a 'background' task scheduled by the top half to
* take care of the expensive tasks of the interrupt processing.
*
* The driver shutdown code calls #sah_Intr_Release().
*
*/

#include <portable_os.h>

/* SAHARA Includes */
#include <sah_kernel.h>
#include <sah_interrupt_handler.h>
#include <sah_status_manager.h>
#include <sah_hardware_interface.h>
#include <sah_queue_manager.h>

/*Enable this flag for debugging*/
#if 0
#define DIAG_DRV_INTERRUPT
#endif

#ifdef DIAG_DRV_INTERRUPT
#include <diagnostic.h>
#endif

/*!
 * Number of interrupts received.  This value should only be updated during
 * interrupt processing.
 */
uint32_t interrupt_count;

#ifndef SAHARA_POLL_MODE

#if !defined(LINUX_VERSION_CODE) || LINUX_VERSION_CODE < KERNEL_VERSION(2,5,0)
#define irqreturn_t void
#define IRQ_HANDLED
#define IRQ_RETVAL(x)
#endif

/* Internal Prototypes */
static irqreturn_t sah_Intr_Top_Half(int irq, void *dev_id);

#ifdef KERNEL_TEST
extern void (*SAHARA_INT_PTR) (int, void *);
#endif

unsigned long reset_flag;
static void sah_Intr_Bottom_Half(unsigned long reset_flag);

/* This is the Bottom Half Task, (reset flag set to false) */
DECLARE_TASKLET(BH_task, sah_Intr_Bottom_Half, (unsigned long)&reset_flag);

/*! This is set by the Initialisation function */
wait_queue_head_t *int_queue = NULL;

/*!
*******************************************************************************
* This function registers the Top Half of the interrupt handler with the Kernel
* and the SAHARA IRQ number.
*
* @brief     SAHARA Interrupt Handler Initialisation
*
* @param    wait_queue Pointer to the wait queue used by driver interface
*
* @return   int A return of Zero indicates successful initialisation.
*/
/******************************************************************************
*
* CAUTION: NONE
*
* MODIFICATION HISTORY:
*
* Date         Person     Change
* 30/07/2003   MW         Initial Creation
******************************************************************************/
int sah_Intr_Init(wait_queue_head_t * wait_queue)
{

#ifdef DIAG_DRV_INTERRUPT
	char err_string[200];
#endif

	int result;

#ifdef KERNEL_TEST
	SAHARA_INT_PTR = sah_Intr_Top_Half;
#endif

	/* Set queue used by the interrupt handler to match the driver interface */
	int_queue = wait_queue;

	/* Request use of the Interrupt line. */
	result = request_irq(SAHARA_IRQ,
			     sah_Intr_Top_Half, 0, SAHARA_NAME, NULL);

#ifdef DIAG_DRV_INTERRUPT
	if (result != 0) {
		sprintf(err_string, "Cannot use SAHARA interrupt line %d. "
			"request_irq() return code is %i.", SAHARA_IRQ, result);
		LOG_KDIAG(err_string);
	} else {
		sprintf(err_string,
			"SAHARA driver registered for interrupt %d. ",
			SAHARA_IRQ);
		LOG_KDIAG(err_string);
	}
#endif

	return result;
}

/*!
*******************************************************************************
* This function releases the Top Half of the interrupt handler. The driver will
* not receive any more interrupts after calling this functions.
*
* @brief     SAHARA Interrupt Handler Release
*
* @return   void
*/
/******************************************************************************
*
* CAUTION: NONE
*
* MODIFICATION HISTORY:
*
* Date         Person     Change
* 30/07/2003   MW         Initial Creation
******************************************************************************/
void sah_Intr_Release(void)
{
	/* Release the Interrupt. */
	free_irq(SAHARA_IRQ, NULL);
}

/*!
*******************************************************************************
* This function is the Top Half of the interrupt handler.  It updates the
* status of any finished descriptor chains and then tries to add any pending
* requests into the hardware.  It then queues the bottom half to complete
* operations on the finished chains.
*
* @brief     SAHARA Interrupt Handler Top Half
*
* @param    irq     Part of the kernel prototype.
* @param    dev_id  Part of the kernel prototype.
*
* @return   An IRQ_RETVAL() -- non-zero to that function means 'handled'
*/
static irqreturn_t sah_Intr_Top_Half(int irq, void *dev_id)
{
#if defined(DIAG_DRV_INTERRUPT) && defined(DIAG_DURING_INTERRUPT)
	LOG_KDIAG("Top half of Sahara's interrupt handler called.");
#endif

	interrupt_count++;
	reset_flag = sah_Handle_Interrupt(sah_HW_Read_Status());

	/* Schedule the Bottom Half of the Interrupt. */
	tasklet_schedule(&BH_task);

	/* To get rid of the unused parameter warnings. */
	irq = 0;
	dev_id = NULL;
	return IRQ_RETVAL(1);
}

/*!
*******************************************************************************
* This function is the Bottom Half of the interrupt handler.  It calls
* #sah_postprocess_queue() to complete the processing of the Descriptor Chains
* which were finished by the hardware.
*
* @brief     SAHARA Interrupt Handler Bottom Half
*
* @param    data   Part of the kernel prototype.
*
* @return   void
*/
static void sah_Intr_Bottom_Half(unsigned long reset_flag)
{
#if defined(DIAG_DRV_INTERRUPT) && defined(DIAG_DURING_INTERRUPT)
	LOG_KDIAG("Bottom half of Sahara's interrupt handler called.");
#endif
	sah_postprocess_queue(*(unsigned long *)reset_flag);

	return;
}

/* end of sah_interrupt_handler.c */
#endif				/* ifndef SAHARA_POLL_MODE */
