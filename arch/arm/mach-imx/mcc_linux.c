/*
 * This file contains Linux-specific MCC library functions
 *
 * Copyright (C) 2014 Freescale Semiconductor, Inc. All Rights Reserved.
 *
 *
 * SPDX-License-Identifier: GPL-2.0+ and/or BSD-3-Clause
 * The GPL-2.0+ license for this file can be found in the COPYING.GPL file
 * included with this distribution or at
 * http://www.gnu.org/licenses/gpl-2.0.html
 * The BSD-3-Clause License for this file can be found in the COPYING.BSD file
 * included with this distribution or at
 * http://opensource.org/licenses/BSD-3-Clause
 */

#include <linux/busfreq-imx6.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/mfd/syscon.h>
#include <linux/regmap.h>
#include <linux/sched.h>
#include <linux/wait.h>
#include <linux/imx_sema4.h>
#include <linux/irq.h>
#include "mcc_config.h"
#include <linux/mcc_config_linux.h>
#include <linux/mcc_common.h>
#include <linux/mcc_api.h>
#include <linux/mcc_imx6sx.h>
#include <linux/mcc_linux.h>
#include "common.h"

/* Global variables */
struct regmap *imx_mu_reg;
unsigned long mcc_shm_offset;
static struct imx_sema4_mutex *shm_ptr;
static unsigned int cpu_to_cpu_isr_vector = MCC_VECTOR_NUMBER_INVALID;
static struct delayed_work mu_work;
static u32 m4_wake_irqs[4];
unsigned int m4_message;
bool m4_freq_low;

unsigned int imx_mcc_buffer_freed = 0, imx_mcc_buffer_queued = 0;
DECLARE_WAIT_QUEUE_HEAD(buffer_freed_wait_queue); /* Used for blocking send */
DECLARE_WAIT_QUEUE_HEAD(buffer_queued_wait_queue); /* Used for blocking recv */

MCC_BOOKEEPING_STRUCT *bookeeping_data;

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
	unsigned int irqs = mcc_get_mu_irq();

	if (irqs & (1 << 31)) {
		/*
		 * Try to lock the core mutex. If successfully locked, perform
		 * mcc_dequeue_signal(), release the gate and finally clear the
		 * interrupt flag. If trylock fails (HW semaphore already locked
		 * by another core), do not clear the interrupt flag  this
		 * way the CPU-to-CPU isr is re-issued again until the HW
		 * semaphore is locked. Higher priority ISRs will be serviced
		 * while issued at the time we are waiting for the unlocked
		 * gate. To prevent trylog failure due to core mutex currently
		 * locked by our own core(a task), the cpu-to-cpu isr is
		 * temporarily disabled when mcc_get_semaphore() is called and
		 * re-enabled again when mcc_release_semaphore() is issued.
		 */
		if (SEMA4_A9_LOCK == imx_sema4_mutex_trylock(shm_ptr)) {
			while (MCC_SUCCESS == mcc_dequeue_signal(
				MCC_CORE_NUMBER, &serviced_signal)) {
				if ((serviced_signal.type == BUFFER_QUEUED) &&
					(serviced_signal.destination.core ==
					MCC_CORE_NUMBER)) {
					/*
					 * Unblock receiver, in case of
					 * asynchronous communication
					 */
					imx_mcc_buffer_queued = 1;
					wake_up(&buffer_queued_wait_queue);
				} else if (serviced_signal.type ==
					BUFFER_FREED) {
					/*
					 * Unblock sender, in case of
					 * asynchronous communication
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
	}

	if (irqs & (1 << 27)) {
		m4_message = mcc_handle_mu_receive_irq();
		schedule_delayed_work(&mu_work, 0);
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

void mcc_enable_m4_irqs_in_gic(bool enable)
{
	int i, j;

	for (i = 0; i < 4; i++) {
		if (m4_wake_irqs[i] == 0)
			continue;
		for (j = 0; j < 32; j++) {
			if (m4_wake_irqs[i] & (1 << j)) {
				if (enable)
					enable_irq((i + 1) * 32 + j);
				else
					disable_irq((i + 1) * 32 + j);
			}
		}
	}
}

static irqreturn_t mcc_m4_dummy_isr(int irq, void *param)
{
	return IRQ_HANDLED;
}

static void mu_work_handler(struct work_struct *work)
{
	int ret;
	u32 irq, enable, idx, mask;

	pr_debug("receive M4 message 0x%x\n", m4_message);

	switch (m4_message) {
	case MU_LPM_M4_REQUEST_HIGH_BUS:
		request_bus_freq(BUS_FREQ_HIGH);
		imx6sx_set_m4_highfreq(true);
		mcc_send_via_mu_buffer(MU_LPM_HANDSHAKE_INDEX,
			MU_LPM_BUS_HIGH_READY_FOR_M4);
		m4_freq_low = false;
		break;
	case MU_LPM_M4_RELEASE_HIGH_BUS:
		release_bus_freq(BUS_FREQ_HIGH);
		imx6sx_set_m4_highfreq(false);
		mcc_send_via_mu_buffer(MU_LPM_HANDSHAKE_INDEX,
			MU_LPM_M4_FREQ_CHANGE_READY);
		m4_freq_low = true;
		break;
	default:
		if ((m4_message & MU_LPM_M4_WAKEUP_SRC_MASK) ==
			MU_LPM_M4_WAKEUP_SRC_VAL) {
			irq = (m4_message & MU_LPM_M4_WAKEUP_IRQ_MASK) >>
				MU_LPM_M4_WAKEUP_IRQ_SHIFT;

			enable = (m4_message & MU_LPM_M4_WAKEUP_ENABLE_MASK) >>
				MU_LPM_M4_WAKEUP_ENABLE_SHIFT;

			idx = irq / 32 - 1;
			mask = 1 << irq % 32;

			if (enable && can_request_irq(irq, 0)) {

				ret = request_irq(irq, mcc_m4_dummy_isr,
					IRQF_NO_SUSPEND, "imx-m4-dummy", NULL);
				if (ret) {
					pr_err("%s: register interrupt %d failed, rc %d\n",
						__func__, irq, ret);
					break;
				}
				disable_irq(irq);
				m4_wake_irqs[idx] = m4_wake_irqs[idx] | mask;
			}
			imx_gpc_add_m4_wake_up_irq(irq, enable);
		}
		break;
	}
	m4_message = 0;
	mcc_enable_receive_irq(1);
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
				IRQF_EARLY_RESUME, "imx-linux-mcc", NULL);
		if (ret) {
			pr_err("%s: register interrupt %d failed, rc %d\n",
					__func__, vector_number, ret);
			return MCC_ERR_INT;
		}

		INIT_DELAYED_WORK(&mu_work, mu_work_handler);
		/* Make sure the INT is cleared */
		mcc_clear_cpu_to_cpu_interrupt(MCC_CORE_NUMBER);

		/* Enable INT */
		ret = imx_mcc_bsp_int_enable(vector_number);
		if (ret < 0) {
			pr_err("ERR:failed to enable mcc int.\n");
			return MCC_ERR_INT;
		}
		/* MU always as a wakeup source for low power mode */
		imx_gpc_add_m4_wake_up_irq(vector_number, true);

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
	int ret;

	/*
	 * Assert directed CPU interrupts for all processors except
	 * the requesting core
	 */
	ret = mcc_triger_cpu_to_cpu_interrupt();

	if (ret == 0)
		return MCC_SUCCESS;
	else
		return ret;
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

void *mcc_virt_to_phys(void *x)
{
	if (null == x)
		return NULL;
	else
		return (void *)((unsigned long) (x) - mcc_shm_offset);
}

void *mcc_phys_to_virt(void *x)
{
	if (null == x)
		return NULL;
	else
		return (void *)((unsigned long) (x) + mcc_shm_offset);
}

int mcc_init_os_sync(void)
{
	/* No used in linux */
	return MCC_SUCCESS;
}

int mcc_deinit_os_sync(void)
{
	/* No used in linux */
	return MCC_SUCCESS;
}

void mcc_clear_os_sync_for_ep(MCC_ENDPOINT *endpoint)
{
	/* No used in linux */
}

int mcc_wait_for_buffer_freed(MCC_RECEIVE_BUFFER **buffer, unsigned int timeout)
{
    int return_value;
    unsigned long timeout_j; /* jiffies */
    MCC_RECEIVE_BUFFER *buf = null;

	/*
	 * Blocking calls: CPU-to-CPU ISR sets the event and thus
	 * resumes tasks waiting for a free MCC buffer.
	 * As the interrupt request is send to all cores when a buffer
	 * is freed it could happen that several tasks from different
	 * cores/nodes are waiting for a free buffer and all of them
	 * are notified that the buffer has been freed. This function
	 * has to check (after the wake up) that a buffer is really
	 * available and has not been already grabbed by another
	 * "competitor task" that has been faster. If so, it has to
	 * wait again for the next notification.
	 */
	while (buf == null) {
		if (timeout == 0xffffffff) {
			/*
			 * In order to level up the robust, do not always
			 * wait event here. Wake up itself after every 1~s.
			 */
			timeout_j = usecs_to_jiffies(1000);
			wait_event_timeout(buffer_freed_wait_queue,
					imx_mcc_buffer_freed == 1, timeout_j);
		} else {
			timeout_j = msecs_to_jiffies(timeout);
			wait_event_timeout(buffer_freed_wait_queue,
					imx_mcc_buffer_freed == 1, timeout_j);
		}

		return_value = mcc_get_semaphore();
		if (return_value != MCC_SUCCESS)
			return return_value;

		MCC_DCACHE_INVALIDATE_MLINES((void *)
				&bookeeping_data->free_list,
				sizeof(MCC_RECEIVE_LIST *));

		buf = mcc_dequeue_buffer(&bookeeping_data->free_list);
		mcc_release_semaphore();
		if (imx_mcc_buffer_freed)
			imx_mcc_buffer_freed = 0;
	}

	*buffer = buf;
	return MCC_SUCCESS;
}

int mcc_wait_for_buffer_queued(MCC_ENDPOINT *endpoint, unsigned int timeout)
{
	unsigned long timeout_j; /* jiffies */
	MCC_RECEIVE_LIST *tmp_list;

	/* Get list of buffers kept by the particular endpoint */
	tmp_list = mcc_get_endpoint_list(*endpoint);

	if (timeout == 0xffffffff) {
		wait_event(buffer_queued_wait_queue,
				imx_mcc_buffer_queued == 1);
		mcc_get_semaphore();
		/*
		* double check if the tmp_list head is still null
		* or not, if yes, wait again.
		*/
		while (tmp_list->head == null) {
			imx_mcc_buffer_queued = 0;
			mcc_release_semaphore();
			wait_event(buffer_queued_wait_queue,
					imx_mcc_buffer_queued == 1);
			mcc_get_semaphore();
		}
	} else {
		timeout_j = msecs_to_jiffies(timeout);
		wait_event_timeout(buffer_queued_wait_queue,
				imx_mcc_buffer_queued == 1, timeout_j);
		mcc_get_semaphore();
	}

	if (imx_mcc_buffer_queued)
		imx_mcc_buffer_queued = 0;

	if (tmp_list->head == null) {
		pr_err("%s can't get queued buffer.\n", __func__);
		mcc_release_semaphore();
		return MCC_ERR_TIMEOUT;
	}

	tmp_list->head = (MCC_RECEIVE_BUFFER *)
		MCC_MEM_PHYS_TO_VIRT(tmp_list->head);
	mcc_release_semaphore();

	return MCC_SUCCESS;
}

MCC_BOOKEEPING_STRUCT *mcc_get_bookeeping_data(void)
{
	MCC_BOOKEEPING_STRUCT *bookeeping_data;

	bookeeping_data = (MCC_BOOKEEPING_STRUCT *)ioremap_nocache
		(MCC_BASE_ADDRESS, sizeof(struct mcc_bookeeping_struct));
	mcc_shm_offset = (unsigned long)bookeeping_data
		- (unsigned long)MCC_BASE_ADDRESS;

	return bookeeping_data;
}
