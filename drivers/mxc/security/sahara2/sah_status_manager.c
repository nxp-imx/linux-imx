/*
 * Copyright (C) 2004-2011 Freescale Semiconductor, Inc. All Rights Reserved.
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
* @file sah_status_manager.c
*
* @brief Status Manager Function
*
* This file contains the function which processes the Sahara status register
* during an interrupt.
*
* This file does not need porting.
*/

#include "portable_os.h"

#include <sah_status_manager.h>
#include <sah_hardware_interface.h>
#include <sah_queue_manager.h>
#include <sah_memory_mapper.h>
#include <sah_kernel.h>

#if defined(DIAG_DRV_INTERRUPT) && defined(DIAG_DURING_INTERRUPT)
#include <diagnostic.h>
#endif

/*! Compile-time flag to count various interrupt types. */
#define DIAG_INT_COUNT

/*!
 * Number of interrupts processed with Done1Done2 status.  Updates to this
 * value should only be done in interrupt processing.
 */
uint32_t done1_count;

/*!
 * Number of interrupts processed with Done1Busy2 status.  Updates to this
 * value should only be done in interrupt processing.
 */
uint32_t done1busy2_count;

/*!
 * Number of interrupts processed with Done1Done2 status.  Updates to this
 * value should only be done in interrupt processing.
 */
uint32_t done1done2_count;

/*!
 * the dynameic power management flag is false when power management is not
 * asserted and true when dpm is.
 */
#ifdef SAHARA_POWER_MANAGEMENT
bool sah_dpm_flag = FALSE;

#if  LINUX_VERSION_CODE < KERNEL_VERSION(2,6,11)
static int sah_dpm_suspend(struct device *dev, uint32_t state, uint32_t level);
static int sah_dpm_resume(struct device *dev, uint32_t level);
#else
static int sah_dpm_suspend(struct platform_device *dev, pm_message_t state);
static int sah_dpm_resume(struct platform_device *dev);
#endif
#endif

#ifndef SAHARA_POLL_MODE
/*!
*******************************************************************************
* This functionx processes the status register of the Sahara, updates the state
* of the finished queue entry, and then tries to find more work for Sahara to
* do.
*
* @brief    The bulk of the interrupt handling code.
*
* @param   hw_status     The status register of Sahara at time of interrupt.
*                        The Clear interrupt bit is already handled by this
*                        register read prior to entry into this function.
* @return   void
*/
unsigned long sah_Handle_Interrupt(sah_Execute_Status hw_status)
{
	unsigned long reset_flag = 0;	/* assume no SAHARA reset needed */
	os_lock_context_t lock_flags;
	sah_Head_Desc *current_entry;

	/* HW status at time of interrupt */
	sah_Execute_Status state = hw_status & SAH_EXEC_STATE_MASK;

	do {
		uint32_t dar;

#ifdef DIAG_INT_COUNT
		if (state == SAH_EXEC_DONE1) {
			done1_count++;
		} else if (state == SAH_EXEC_DONE1_BUSY2) {
			done1busy2_count++;
		} else if (state == SAH_EXEC_DONE1_DONE2) {
			done1done2_count++;
		}
#endif

		/* if the first entry on sahara has completed... */
		if ((state & SAH_EXEC_DONE1_BIT) ||
		    (state == SAH_EXEC_ERROR1)) {
			/* lock queue while searching */
			os_lock_save_context(desc_queue_lock, lock_flags);
			current_entry =
			    sah_Find_With_State(SAH_STATE_ON_SAHARA);
			os_unlock_restore_context(desc_queue_lock, lock_flags);

			/* an active descriptor was not found */
			if (current_entry == NULL) {
				/* change state to avoid an infinite loop (possible if
				 * state is SAH_EXEC_DONE1_BUSY2 first time into loop) */
				hw_status = SAH_EXEC_IDLE;
#if defined(DIAG_DRV_INTERRUPT) && defined(DIAG_DURING_INTERRUPT)
				LOG_KDIAG
				    ("Interrupt received with nothing on queue.");
#endif
			} else {
				/* SAHARA has completed its work on this descriptor chain */
				current_entry->status = SAH_STATE_OFF_SAHARA;

				if (state == SAH_EXEC_ERROR1) {
				   if (hw_status & STATUS_ERROR) {
					  /* Gather extra diagnostic information */
					  current_entry->fault_address =
					    sah_HW_Read_Fault_Address();
					  /* Read this last - it clears the error */
					  current_entry->error_status =
					    sah_HW_Read_Error_Status();
					  current_entry->op_status = 0;
#ifdef FSL_HAVE_SAHARA4
				   } else {
					  current_entry->op_status =
							sah_HW_Read_Op_Status();
					  current_entry->error_status = 0;
#endif
				   }

				} else {
					/* indicate that no errors were found with descriptor
					 * chain 1 */
					current_entry->error_status = 0;
					current_entry->op_status = 0;

					/* is there a second, successfully, completed descriptor
					 * chain? (done1/error2 processing is handled later) */
					if (state == SAH_EXEC_DONE1_DONE2) {
						os_lock_save_context
						    (desc_queue_lock,
						     lock_flags);
						current_entry =
						    sah_Find_With_State
						    (SAH_STATE_ON_SAHARA);
						os_unlock_restore_context
						    (desc_queue_lock,
						     lock_flags);

						if (current_entry == NULL) {
#if defined(DIAG_DRV_INTERRUPT) && defined(DIAG_DURING_INTERRUPT)
							LOG_KDIAG
							    ("Done1_Done2 Interrupt received with "
							     "one entry on queue.");
#endif
						} else {
							/* indicate no errors in descriptor chain 2 */
							current_entry->
							    error_status = 0;
							current_entry->status =
							    SAH_STATE_OFF_SAHARA;
						}
					}
				}
			}

#ifdef SAHARA_POWER_MANAGEMENT
			/* check dynamic power management is not asserted */
			if (!sah_dpm_flag) {
#endif
				do {
					/* protect DAR and main_queue */
					os_lock_save_context(desc_queue_lock,
							     lock_flags);
					dar = sah_HW_Read_DAR();
					/* check if SAHARA has space for another descriptor. SAHARA
					 * only accepts up to the DAR queue size number of DAR
					 * entries, after that 'dar' will not be zero until the
					 * pending interrupt is serviced */
					if (dar == 0) {
						current_entry =
						    sah_Find_With_State
						    (SAH_STATE_PENDING);
						if (current_entry != NULL) {
#ifndef SUBMIT_MULTIPLE_DARS
							/* BUG FIX: state machine can transition from Done1
							 * Busy2 directly to Idle. To fix that problem,
							 * only one DAR is being allowed on SAHARA at a
							 * time.  If a high level interrupt has happened,
							 * there could * be an active descriptor chain */
							if (sah_Find_With_State
							    (SAH_STATE_ON_SAHARA)
							    == NULL) {
#endif
#if defined(DIAG_DRV_IF) && defined(DIAG_DURING_INTERRUPT)
								sah_Dump_Chain
								    (&current_entry->
								     desc,
								     current_entry->
								     desc.
								     dma_addr);
#endif				/* DIAG_DRV_IF */
								sah_HW_Write_DAR
								    (current_entry->
								     desc.
								     dma_addr);
								current_entry->
								    status =
								    SAH_STATE_ON_SAHARA;
#ifndef SUBMIT_MULTIPLE_DARS
							}
							current_entry = NULL;	/* exit loop */
#endif
						}
					}
					os_unlock_restore_context
					    (desc_queue_lock, lock_flags);
				} while ((dar == 0) && (current_entry != NULL));
#ifdef SAHARA_POWER_MANAGEMENT
			}	/* sah_device_power_manager */
#endif
		} else {
			if (state == SAH_EXEC_FAULT) {
				sah_Head_Desc *previous_entry;	/* point to chain 1 */
				/* Address of request when fault occured */
				uint32_t bad_dar = sah_HW_Read_IDAR();

				reset_flag = 1;	/* SAHARA needs to be reset */

				/* get first of possible two descriptor chain that was
				 * on SAHARA */
				os_lock_save_context(desc_queue_lock,
						     lock_flags);
				previous_entry =
				    sah_Find_With_State(SAH_STATE_ON_SAHARA);
				os_unlock_restore_context(desc_queue_lock,
							  lock_flags);

				/* if it exists, continue processing the fault */
				if (previous_entry) {
					/* assume this chain didn't complete correctly */
					previous_entry->error_status = -1;
					previous_entry->status =
					    SAH_STATE_OFF_SAHARA;

					/* get the second descriptor chain */
					os_lock_save_context(desc_queue_lock,
							     lock_flags);
					current_entry =
					    sah_Find_With_State
					    (SAH_STATE_ON_SAHARA);
					os_unlock_restore_context
					    (desc_queue_lock, lock_flags);

					/* if it exists, continue processing both chains */
					if (current_entry) {
						/* assume this chain didn't complete correctly */
						current_entry->error_status =
						    -1;
						current_entry->status =
						    SAH_STATE_OFF_SAHARA;

						/* now see if either can be identified as the one
						 * in progress when the fault occured */
						if (current_entry->desc.
						    dma_addr == bad_dar) {
							/* the second descriptor chain was active when the
							 * fault occured, so the first descriptor chain
							 * was successfull */
							previous_entry->
							    error_status = 0;
						} else {
							if (previous_entry->
							    desc.dma_addr ==
							    bad_dar) {
								/* if the first chain was in progress when the
								 * fault occured, the second has not yet been
								 * touched, so reset it to PENDING */
								current_entry->
								    status =
								    SAH_STATE_PENDING;
							}
						}
					}
				}
#if defined(DIAG_DRV_INTERRUPT) && defined(DIAG_DURING_INTERRUPT)
			} else {
				/* shouldn't ever get here */
				if (state == SAH_EXEC_BUSY) {
					LOG_KDIAG
					    ("Got Sahara interrupt in Busy state");
				} else {
					if (state == SAH_EXEC_IDLE) {
						LOG_KDIAG
						    ("Got Sahara interrupt in Idle state");
					} else {
						LOG_KDIAG
						    ("Got Sahara interrupt in unknown state");
					}
				}
#endif
			}
		}

		/* haven't handled the done1/error2 (the error 2 part), so setup to
		 * do that now. Otherwise, exit loop */
		state = (state == SAH_EXEC_DONE1_ERROR2) ?
		    SAH_EXEC_ERROR1 : SAH_EXEC_IDLE;

		/* Keep going while further status is available. */
	} while (state == SAH_EXEC_ERROR1);

	return reset_flag;
}

#endif				/* ifndef SAHARA_POLL_MODE */

#ifdef SAHARA_POLL_MODE
/*!
*******************************************************************************
* Submits descriptor chain to SAHARA, polls on SAHARA for completion, process
* results, and dephysicalizes chain
*
* @brief     Handle poll mode.
*
* @param    entry   Virtual address of a physicalized chain
*
* @return   0      this function is always successful
*/

unsigned long sah_Handle_Poll(sah_Head_Desc * entry)
{
	sah_Execute_Status hw_status;	/* Sahara's status register */
	os_lock_context_t lock_flags;

	/* lock SARAHA */
	os_lock_save_context(desc_queue_lock, lock_flags);

#ifdef SAHARA_POWER_MANAGEMENT
	/* check if the dynamic power management is asserted */
	if (sah_dpm_flag) {
		/* return that request failed to be processed */
		entry->result = FSL_RETURN_ERROR_S;
		entry->fault_address = 0xBAD;
		entry->op_status= 0xBAD;
		entry->error_status = 0xBAD;
	} else {
#endif				/* SAHARA_POWER_MANAGEMENT */

#if defined(DIAG_DRV_IF)
		sah_Dump_Chain(&entry->desc, entry->desc.dma_addr);
#endif				/* DIAG_DRV_IF */
		/* Nothing can be in the dar if we got the lock */
		sah_HW_Write_DAR((uint32_t) (entry->desc.dma_addr));

		/* Wait for SAHARA to finish with this entry */
		hw_status = sah_Wait_On_Sahara();

		/* if entry completed successfully, mark it as such */
	/**** HARDWARE ERROR WORK AROUND (hw_status == SAH_EXEC_IDLE) *****/
		if (
#ifndef SUBMIT_MULTIPLE_DARS
			   (hw_status == SAH_EXEC_IDLE) || (hw_status == SAH_EXEC_DONE1_BUSY2) ||	/* should not happen */
#endif
			   (hw_status == SAH_EXEC_DONE1)
		    ) {
			entry->error_status = 0;
			entry->result = FSL_RETURN_OK_S;
		} else {
			/* SAHARA is reporting an error with entry */
			if (hw_status == SAH_EXEC_ERROR1) {
				/* Gather extra diagnostic information */
				entry->fault_address =
				    sah_HW_Read_Fault_Address();
				/* Read this register last - it clears the error */
				entry->error_status =
				    sah_HW_Read_Error_Status();
				entry->op_status = 0;
				/* translate from SAHARA error status to fsl_shw return values */
				entry->result =
				    sah_convert_error_status(entry->
							     error_status);
#ifdef DIAG_DRV_STATUS
				sah_Log_Error(entry->op_status,
					      entry->error_status,
					      entry->fault_address);
#endif
			} else if (hw_status == SAH_EXEC_OPSTAT1) {
				entry->op_status = sah_HW_Read_Op_Status();
				entry->error_status = 0;
				entry->result =
				    sah_convert_op_status(op_status);
			} else {
				/* SAHARA entered FAULT state (or something bazaar has
				 * happened) */
				pr_debug
					("Sahara: hw_status = 0x%x; Stat: 0x%08x; IDAR: 0x%08x; "
					 "CDAR: 0x%08x; FltAdr: 0x%08x; Estat: 0x%08x\n",
					 hw_status, sah_HW_Read_Status(),
					 sah_HW_Read_IDAR(), sah_HW_Read_CDAR(),
					 sah_HW_Read_Fault_Address(),
					 sah_HW_Read_Error_Status());
#ifdef DIAG_DRV_IF
				{
					int old_level = console_loglevel;
					console_loglevel = 8;
					sah_Dump_Chain(&(entry->desc),
							   entry->desc.dma_addr);
					console_loglevel = old_level;
				}
#endif

				entry->error_status = -1;
				entry->result = FSL_RETURN_ERROR_S;
				sah_HW_Reset();
			}
		}
#ifdef SAHARA_POWER_MANAGEMENT
	}
#endif

	if (!(entry->uco_flags & FSL_UCO_BLOCKING_MODE)) {
		/* put it in results pool to allow get_results to work */
		sah_Queue_Append_Entry(&entry->user_info->result_pool, entry);
		if (entry->uco_flags & FSL_UCO_CALLBACK_MODE) {
			/* invoke callback */
			entry->user_info->callback(entry->user_info);
		}
	} else {
		/* convert the descriptor link back to virtual memory, mark dirty pages
		 * if they are from user mode, and release the page cache for user
		 * pages
		 */
		entry = sah_DePhysicalise_Descriptors(entry);
	}

	os_unlock_restore_context(desc_queue_lock, lock_flags);

	return 0;
}

#endif				/* SAHARA_POLL_MODE */

/******************************************************************************
* The following is the implementation of the Dynamic Power Management
* functionality.
******************************************************************************/
#ifdef SAHARA_POWER_MANAGEMENT

static bool sah_dpm_init_flag;

/* dynamic power management information for the sahara driver */
#if  LINUX_VERSION_CODE < KERNEL_VERSION(2,6,11)
static struct device_driver sah_dpm_driver = {
	.name = "sahara_",
	.bus = &platform_bus_type,
#else
static struct platform_driver sah_dpm_driver = {
	.driver.name = "sahara_",
	.driver.bus = &platform_bus_type,
#endif
	.suspend = sah_dpm_suspend,
	.resume = sah_dpm_resume
};

/* dynamic power management information for the sahara HW device */
static struct platform_device sah_dpm_device = {
	.name = "sahara_",
	.id = 1,
};

/*!
*******************************************************************************
* Initilaizes the dynamic power managment functionality
*
* @brief    Initialization of the Dynamic Power Management functionality
*
* @return   0 = success; failed otherwise
*/
int sah_dpm_init()
{
	int status;

	/* dpm is not asserted */
	sah_dpm_flag = FALSE;

	/* register the driver to the kernel */
#if  LINUX_VERSION_CODE < KERNEL_VERSION(2,6,11)
	status = os_register_to_driver(&sah_dpm_driver);
#else
	status = os_register_to_driver(&sah_dpm_driver.driver);
#endif

	if (status == 0) {
		/* register a single sahara chip */
		/*status = platform_device_register(&sah_dpm_device); */
		status = os_register_a_device(&sah_dpm_device);

		/* if something went awry, unregister the driver */
		if (status != 0) {
			/*driver_unregister(&sah_dpm_driver); */
#if  LINUX_VERSION_CODE < KERNEL_VERSION(2,6,11)
			os_unregister_from_driver(&sah_dpm_driver);
#else
			os_unregister_from_driver(&sah_dpm_driver.driver);
#endif
			sah_dpm_init_flag = FALSE;
		} else {
			/* if everything went okay, flag that life is good */
			sah_dpm_init_flag = TRUE;
		}
	}

	/* let the kernel know how it went */
	return status;

}

/*!
*******************************************************************************
* Unregister the dynamic power managment functionality
*
* @brief    Unregister the Dynamic Power Management functionality
*
*/
void sah_dpm_close()
{
	/* if dynamic power management was initilaized, kill it */
	if (sah_dpm_init_flag == TRUE) {
		/*driver_unregister(&sah_dpm_driver); */
#if  LINUX_VERSION_CODE < KERNEL_VERSION(2,6,11)
		os_unregister_from_driver(&sah_dpm_driver);
#else
		os_unregister_from_driver(&sah_dpm_driver.driver);
#endif
		/*platform_device_register(&sah_dpm_device); */
		os_unregister_a_device(&sah_dpm_device);
	}
}

/*!
*******************************************************************************
* Callback routine defined by the Linux Device Model / Dynamic Power management
* extension. It sets a global flag to disallow the driver to enter queued items
* into Sahara's DAR.
*
* It allows the current active descriptor chains to complete before it returns
*
* @brief   Suspends the driver
*
* @param   dev   contains device information
* @param   state contains state information
* @param   level  level of shutdown
*
* @return   0 = success; failed otherwise
*/

#if  LINUX_VERSION_CODE < KERNEL_VERSION(2,6,11)
static int sah_dpm_suspend(struct device *dev, uint32_t state, uint32_t level)
#else
static int sah_dpm_suspend(struct platform_device *dev, pm_message_t state)
#endif
{
	sah_Head_Desc *entry = NULL;
	os_lock_context_t lock_flags;
	int error_code = 0;

#if  LINUX_VERSION_CODE < KERNEL_VERSION(2,6,11)
	switch (level) {
	case SUSPEND_DISABLE:
		/* Assert dynamic power management. This stops the driver from
		 * entering queued requests to Sahara */
		sah_dpm_flag = TRUE;
		break;

	case SUSPEND_SAVE_STATE:
		break;

	case SUSPEND_POWER_DOWN:
		/* hopefully between the DISABLE call and this one, the outstanding
		 * work Sahara was doing complete. this checks (and waits) for
		 * those entries that were already active on Sahara to complete */
		/* lock queue while searching */
		os_lock_save_context(desc_queue_lock, lock_flags);
		do {
			entry = sah_Find_With_State(SAH_STATE_ON_SAHARA);
		} while (entry != NULL);
		os_unlock_restore_context(desc_queue_lock, lock_flags);

		/* now we kill the clock so the control circuitry isn't sucking
		 * any power */
		mxc_clks_disable(SAHARA2_CLK);
		break;
	}
#else
	/* Assert dynamic power management. This stops the driver from
	 * entering queued requests to Sahara */
	sah_dpm_flag = TRUE;

	/* Now wait for any outstanding work Sahara was doing to complete.
	 * this checks (and waits) for
	 * those entries that were already active on Sahara to complete */
	do {
		/* lock queue while searching */
		os_lock_save_context(desc_queue_lock, lock_flags);
		entry = sah_Find_With_State(SAH_STATE_ON_SAHARA);
		os_unlock_restore_context(desc_queue_lock, lock_flags);
	} while (entry != NULL);

	/* now we kill the clock so the control circuitry isn't sucking
	 * any power */
	{
		struct clk *clk = clk_get(NULL, "sahara_clk");
		if (IS_ERR(clk))
			error_code = PTR_ERR(clk);
		else
			clk_disable(clk);
	}
#endif

	return error_code;
}

/*!
*******************************************************************************
* Callback routine defined by the Linux Device Model / Dynamic Power management
* extension. It cleears a global flag to allow the driver to enter queued items
* into Sahara's DAR.
*
* It primes the mechanism to start depleting the queue
*
* @brief   Resumes the driver
*
* @param   dev    contains device information
* @param   level  level of resumption
*
* @return   0 = success; failed otherwise
*/
#if  LINUX_VERSION_CODE < KERNEL_VERSION(2,6,11)
static int sah_dpm_resume(struct device *dev, uint32_t level)
#else
static int sah_dpm_resume(struct platform_device *dev)
#endif
{
	sah_Head_Desc *entry = NULL;
	os_lock_context_t lock_flags;
	int error_code = 0;

#if  LINUX_VERSION_CODE < KERNEL_VERSION(2,6,11)
	switch (level) {
	case RESUME_POWER_ON:
		/* enable Sahara's clock */
		mxc_clks_enable(SAHARA2_CLK);
		break;

	case RESUME_RESTORE_STATE:
		break;

	case RESUME_ENABLE:
		/* Disable dynamic power managment. This allows the driver to put
		 * entries into Sahara's DAR */
		sah_dpm_flag = FALSE;

		/* find a pending entry to prime the pump */
		os_lock_save_context(desc_queue_lock, lock_flags);
		entry = sah_Find_With_State(SAH_STATE_PENDING);
		if (entry != NULL) {
			sah_Queue_Manager_Prime(entry);
		}
		os_unlock_restore_context(desc_queue_lock, lock_flags);
		break;
	}
#else
	{
		/* enable Sahara's clock */
		struct clk *clk = clk_get(NULL, "sahara_clk");

		if (IS_ERR(clk))
			error_code = PTR_ERR(clk);
		else
			clk_disable(clk);
	}
	sah_dpm_flag = FALSE;

	/* find a pending entry to prime the pump */
	os_lock_save_context(desc_queue_lock, lock_flags);
	entry = sah_Find_With_State(SAH_STATE_PENDING);
	if (entry != NULL) {
		sah_Queue_Manager_Prime(entry);
	}
	os_unlock_restore_context(desc_queue_lock, lock_flags);
#endif
	return error_code;
}

#endif				/* SAHARA_POWER_MANAGEMENT */
