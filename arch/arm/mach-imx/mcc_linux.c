/*
 * This file contains Linux-specific MCC library functions
 *
 * Copyright (C) 2014-2015 Freescale Semiconductor, Inc. All Rights Reserved.
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

#include <linux/io.h>
#include <linux/string.h>
#include <linux/wait.h>
#include <linux/imx_sema4.h>
#include "mcc_config.h"
#include <linux/mcc_imx6sx.h>

/* Global variables */
static unsigned long mcc_shm_offset;

MCC_BOOKEEPING_STRUCT *bookeeping_data;

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
	mcc_shm_ptr = imx_sema4_mutex_create(0, sem_num);

	if (NULL == mcc_shm_ptr)
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
	if (0 == imx_sema4_mutex_destroy(mcc_shm_ptr))
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
	if (imx_mcc_bsp_int_disable()) {
		pr_err("ERR:failed to disable mcc int.\n");
		return MCC_ERR_SEMAPHORE;
	}

	if (0 == imx_sema4_mutex_lock(mcc_shm_ptr)) {
		return MCC_SUCCESS;
	} else {
		if (imx_mcc_bsp_int_enable()) {
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
	if (0 == imx_sema4_mutex_unlock(mcc_shm_ptr)) {
		/*
		 * Enable the cpu-to-cpu isr just in case imx_semae_mutex_unlock
		 * function has not woke up another task waiting for the core
		 * mutex.
		 */
		if (mcc_shm_ptr->gate_val != (MCC_CORE_NUMBER + 1))
			imx_mcc_bsp_int_enable();
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
	/*
	 * CPU to CPU ISR had been registered in MU driver.
	 * return success directly.
	 */
	return MCC_SUCCESS;
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

MCC_BOOKEEPING_STRUCT *mcc_get_bookeeping_data(void)
{
	bookeeping_data = (MCC_BOOKEEPING_STRUCT *)ioremap_nocache
		(MCC_BASE_ADDRESS, sizeof(struct mcc_bookeeping_struct));
	mcc_shm_offset = (unsigned long)bookeeping_data
		- (unsigned long)MCC_BASE_ADDRESS;

	return bookeeping_data;
}
