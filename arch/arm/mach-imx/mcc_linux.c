/*
 * Copyright (C) 2014 Freescale Semiconductor, Inc.
 * Freescale IMX Linux-specific MCC implementation.
 * Linux-specific MCC library functions
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program. If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/mfd/syscon.h>
#include <linux/regmap.h>
#include <linux/sched.h>
#include <linux/wait.h>
#include <linux/imx_sema4.h>
#include "mcc_config.h"
#include <linux/mcc_common.h>
#include <linux/mcc_api.h>
#include <linux/mcc_imx6sx.h>
#include <linux/mcc_linux.h>

/* Global variables */
struct regmap *imx_mu_reg;
unsigned long mcc_shm_offset;
static struct imx_sema4_mutex *shm_ptr;
static unsigned int cpu_to_cpu_isr_vector = MCC_VECTOR_NUMBER_INVALID;

unsigned int imx_mcc_buffer_freed = 0, imx_mcc_buffer_queued = 0;
DECLARE_WAIT_QUEUE_HEAD(buffer_freed_wait_queue); /* Used for blocking send */
DECLARE_WAIT_QUEUE_HEAD(buffer_queued_wait_queue); /* Used for blocking recv */

/*!
 * \brief This function is the CPU-to-CPU interrupt handler.
 *
 * Each core can interrupt the other. There are two logical signals:
 * \n - Receive data available for (Node,Port) - signaled when a buffer
 * is queued to a Receive Data Queue.
 * \n - Buffer available - signaled when a buffer is queued to the Free
 * Buffer Queue.
 * \n It is possible that several signals can occur while one interrupt
 * is being processed.
 *  Therefore, a Receive Signal Queue of received signals is also required
 *  - one for each core.
 *  The interrupting core queues to the tail and the interrupted core pulls
 *  from the head.
 *  For a circular file, no semaphore is required since only the sender
 *  modifies the tail and only the receiver modifies the head.
 *
 * \param[in] param Pointer to data being passed to the ISR.
 */
static irqreturn_t mcc_cpu_to_cpu_isr(int irq, void *param)
{
	MCC_SIGNAL serviced_signal;

	/*
	 * Try to lock the core mutex. If successfully locked, perform
	 * mcc_dequeue_signal(), release the gate and finally clear the
	 * interrupt flag. If trylock fails (HW semaphore already locked
	 * by another core), do not clear the interrupt flag  this
	 * way the CPU-to-CPU isr is re-issued again until the HW semaphore
	 * is locked. Higher priority ISRs will be serviced while issued at
	 * the time we are waiting for the unlocked gate. To prevent trylog
	 * failure due to core mutex currently locked by our own core(a task),
	 * the cpu-to-cpu isr is temporarily disabled when mcc_get_semaphore()
	 * is called and re-enabled again when mcc_release_semaphore()
	 * is issued.
	 */
	if (SEMA4_A9_LOCK == imx_sema4_mutex_trylock(shm_ptr)) {
		while (MCC_SUCCESS == mcc_dequeue_signal(MCC_CORE_NUMBER,
					&serviced_signal)) {
			if ((serviced_signal.type == BUFFER_QUEUED) &&
			(serviced_signal.destination.core == MCC_CORE_NUMBER)) {
				/*
				 * Unblock receiver, in case of asynchronous
				 * communication
				 */
				imx_mcc_buffer_queued = 1;
				wake_up(&buffer_queued_wait_queue);
			} else if (serviced_signal.type == BUFFER_FREED) {
				/* Unblock sender, in case of asynchronous
				 * communication
				 */
				imx_mcc_buffer_freed = 1;
				wake_up(&buffer_freed_wait_queue);
			}
		}

		/* Clear the interrupt flag */
		mcc_clear_cpu_to_cpu_interrupt(MCC_CORE_NUMBER);

		/* Unlocks the core mutex */
		imx_sema4_mutex_unlock(shm_ptr);
	}

	return IRQ_HANDLED;
}

/*!
 * \brief This function initializes the hw semaphore (SEMA4).
 *
 * Calls core-mutex driver to create a core mutex.
 *
 * \param[in] sem_num SEMA4 gate number.
 */
int mcc_init_semaphore(unsigned int sem_num)
{
	/* Create a core mutex */
	shm_ptr = imx_sema4_mutex_create(0, sem_num);

	if (NULL == shm_ptr)
		return MCC_ERR_SEMAPHORE;
	else
		return MCC_SUCCESS;
}

/*!
 * \brief This function de-initializes the hw semaphore (SEMA4).
 *
 * Calls core-mutex driver to destroy a core mutex.
 *
 * \param[in] sem_num SEMA4 gate number.
 */
int mcc_deinit_semaphore(unsigned int sem_num)
{
	/* Destroy the core mutex */
	if (0 == imx_sema4_mutex_destroy(shm_ptr))
		return MCC_SUCCESS;
	else
		return MCC_ERR_SEMAPHORE;
}

/*!
 * \brief This function locks the specified core mutex.
 *
 * Calls core-mutex driver to lock the core mutex.
 *
 */
int mcc_get_semaphore(void)
{
	if (imx_mcc_bsp_int_disable(cpu_to_cpu_isr_vector)) {
		pr_err("ERR:failed to disable mcc int.\n");
		return MCC_ERR_SEMAPHORE;
	}

	if (0 == imx_sema4_mutex_lock(shm_ptr)) {
		return MCC_SUCCESS;
	} else {
		if (imx_mcc_bsp_int_enable(cpu_to_cpu_isr_vector)) {
			pr_err("ERR:failed to enable mcc int.\n");
			return MCC_ERR_INT;
		}
		return MCC_ERR_SEMAPHORE;
	}
}

/*!
 * \brief This function unlocks the specified core mutex.
 *
 * Calls core-mutex driver to unlock the core mutex.
 *
 */
int mcc_release_semaphore(void)
{
	if (0 == imx_sema4_mutex_unlock(shm_ptr)) {
		/*
		 * Enable the cpu-to-cpu isr just in case imx_semae_mutex_unlock
		 * function has not woke up another task waiting for the core
		 * mutex.
		 */
		if (shm_ptr->gate_val != (MCC_CORE_NUMBER + 1))
			imx_mcc_bsp_int_enable(cpu_to_cpu_isr_vector);
			return MCC_SUCCESS;
	}

	return MCC_ERR_SEMAPHORE;
}

/*!
 * \brief This function registers the CPU-to-CPU interrupt.
 *
 * Calls interrupt component functions to install and enable the
 * CPU-to-CPU interrupt.
 *
 */
int mcc_register_cpu_to_cpu_isr(void)
{
	int ret;
	unsigned int vector_number;

	vector_number = mcc_get_cpu_to_cpu_vector(MCC_CORE_NUMBER);

	if (vector_number != MCC_VECTOR_NUMBER_INVALID) {
		imx_mu_reg =
			syscon_regmap_lookup_by_compatible("fsl,imx6sx-mu");
		if (IS_ERR(imx_mu_reg))
			pr_err("ERR:unable to find imx mu registers\n");

		ret = request_irq(vector_number, mcc_cpu_to_cpu_isr,
				0, "imx-linux-mcc", NULL);
		if (ret) {
			pr_err("%s: register interrupt %d failed, rc %d\n",
					__func__, vector_number, ret);
			return MCC_ERR_INT;
		}
		/* Make sure the INT is cleared */
		mcc_clear_cpu_to_cpu_interrupt(MCC_CORE_NUMBER);

		/* Enable INT */
		ret = imx_mcc_bsp_int_enable(vector_number);
		if (ret < 0) {
			pr_err("ERR:failed to enable mcc int.\n");
			return MCC_ERR_INT;
		}

		cpu_to_cpu_isr_vector = vector_number;
		return MCC_SUCCESS;
	} else {
		return MCC_ERR_INT;
	}
}

/*!
 * \brief This function triggers an interrupt to other core(s).
 *
 */
int mcc_generate_cpu_to_cpu_interrupt(void)
{
	/*
	 * Assert directed CPU interrupts for all processors except
	 * the requesting core
	 */
	mcc_triger_cpu_to_cpu_interrupt();

	return MCC_SUCCESS;
}

/*!
 * \brief This function copies data.
 *
 * Copies the number of single-addressable units from the source address
 * to destination address.
 *
 * \param[in] src Source address.
 * \param[in] dest Destination address.
 * \param[in] size Number of single-addressable units to copy.
 */
void mcc_memcpy(void *src, void *dest, unsigned int size)
{
	memcpy(dest, src, size);
}

void _mem_zero(void *ptr, unsigned int size)
{
	memset(ptr, 0, size);
}

void *virt_to_mqx(void *x)
{
	if (null == x)
		return NULL;
	else
		return (void *)((unsigned long) (x) - mcc_shm_offset);
}

void *mqx_to_virt(void *x)
{
	if (null == x)
		return NULL;
	else
		return (void *)((unsigned long) (x) + mcc_shm_offset);
}
