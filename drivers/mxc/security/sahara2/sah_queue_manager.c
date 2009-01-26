/*
 * Copyright 2004-2007 Freescale Semiconductor, Inc. All Rights Reserved.
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
 * @file sah_queue_manager.c
 *
 * @brief This file provides a Queue Manager implementation.
 *
 * The Queue Manager manages additions and removal from the queue and updates
 * the status of queue entries. It also calls sah_HW_* functions to interract
 * with the hardware.
*/

#include "portable_os.h"

/* SAHARA Includes */
#include <sah_driver_common.h>
#include <sah_queue_manager.h>
#include <sah_status_manager.h>
#include <sah_hardware_interface.h>
#if defined(DIAG_DRV_QUEUE) || defined(DIAG_DRV_STATUS)
#include <diagnostic.h>
#endif
#include <sah_memory_mapper.h>

#ifdef DIAG_DRV_STATUS

#define FSL_INVALID_RETURN 13
#define MAX_RETURN_STRING_LEN 22
#endif

/* Defines for parsing value from Error Status register */
#define SAH_STATUS_MASK 0x07
#define SAH_ERROR_MASK 0x0F
#define SAH_CHA_ERR_SOURCE_MASK 0x07
#define SAH_CHA_ERR_STATUS_MASK 0x0FFF
#define SAH_DMA_ERR_STATUS_MASK 0x0F
#define SAH_DMA_ERR_SIZE_MASK 0x03
#define SAH_DMA_ERR_DIR_MASK 0x01

#define SHA_ERROR_STATUS_CONTINUE 0xFFFFFFFF
#define SHA_CHA_ERROR_STATUS_DONE 0xFFFFFFFF

/* this maps the error status register's error source 4 bit field to the API
 * return values. A 0xFFFFFFFF indicates additional fields must be checked to
 * determine an appropriate return value */
static sah_Execute_Error sah_Execute_Error_Array[] = {
	FSL_RETURN_ERROR_S,	/* SAH_ERR_NONE */
	FSL_RETURN_BAD_FLAG_S,	/* SAH_ERR_HEADER */
	FSL_RETURN_BAD_DATA_LENGTH_S,	/* SAH_ERR_DESC_LENGTH */
	FSL_RETURN_BAD_DATA_LENGTH_S,	/* SAH_ERR_DESC_POINTER */
	FSL_RETURN_BAD_DATA_LENGTH_S,	/* SAH_ERR_LINK_LENGTH */
	FSL_RETURN_BAD_DATA_LENGTH_S,	/* SAH_ERR_LINK_POINTER */
	FSL_RETURN_INTERNAL_ERROR_S,	/* SAH_ERR_INPUT_BUFFER */
	FSL_RETURN_INTERNAL_ERROR_S,	/* SAH_ERR_OUTPUT_BUFFER */
	FSL_RETURN_BAD_DATA_LENGTH_S,	/* SAH_ERR_OUTPUT_BUFFER_STARVATION */
	FSL_RETURN_INTERNAL_ERROR_S,	/* SAH_ERR_INTERNAL_STATE */
	FSL_RETURN_ERROR_S,	/* SAH_ERR_GENERAL_DESCRIPTOR */
	FSL_RETURN_INTERNAL_ERROR_S,	/* SAH_ERR_RESERVED_FIELDS */
	FSL_RETURN_MEMORY_ERROR_S,	/* SAH_ERR_DESCRIPTOR_ADDRESS */
	FSL_RETURN_MEMORY_ERROR_S,	/* SAH_ERR_LINK_ADDRESS */
	SHA_ERROR_STATUS_CONTINUE,	/* SAH_ERR_CHA */
	SHA_ERROR_STATUS_CONTINUE	/* SAH_ERR_DMA */
};

static sah_DMA_Error_Status sah_DMA_Error_Status_Array[] = {
	FSL_RETURN_INTERNAL_ERROR_S,	/* SAH_DMA_NO_ERR */
	FSL_RETURN_INTERNAL_ERROR_S,	/* SAH_DMA_AHB_ERR */
	FSL_RETURN_INTERNAL_ERROR_S,	/* SAH_DMA_IP_ERR */
	FSL_RETURN_INTERNAL_ERROR_S,	/* SAH_DMA_PARITY_ERR */
	FSL_RETURN_BAD_DATA_LENGTH_S,	/* SAH_DMA_BOUNDRY_ERR */
	FSL_RETURN_INTERNAL_ERROR_S,	/* SAH_DMA_BUSY_ERR */
	FSL_RETURN_INTERNAL_ERROR_S,	/* SAH_DMA_RESERVED_ERR */
	FSL_RETURN_INTERNAL_ERROR_S	/* SAH_DMA_INT_ERR */
};

static sah_CHA_Error_Status sah_CHA_Error_Status_Array[] = {
	FSL_RETURN_INTERNAL_ERROR_S,	/* SAH_CHA_NO_ERR */
	FSL_RETURN_BAD_DATA_LENGTH_S,	/* SAH_CHA_IP_BUF */
	FSL_RETURN_INTERNAL_ERROR_S,	/* SAH_CHA_ADD_ERR */
	FSL_RETURN_BAD_MODE_S,	/* SAH_CHA_MODE_ERR */
	FSL_RETURN_BAD_DATA_LENGTH_S,	/* SAH_CHA_DATA_SIZE_ERR */
	FSL_RETURN_BAD_KEY_LENGTH_S,	/* SAH_CHA_KEY_SIZE_ERR */
	FSL_RETURN_BAD_MODE_S,	/* SAH_CHA_PROC_ERR */
	FSL_RETURN_ERROR_S,	/* SAH_CHA_CTX_READ_ERR */
	FSL_RETURN_INTERNAL_ERROR_S,	/* SAH_CHA_INTERNAL_HW_ERR */
	FSL_RETURN_MEMORY_ERROR_S,	/* SAH_CHA_IP_BUFF_ERR */
	FSL_RETURN_MEMORY_ERROR_S,	/* SAH_CHA_OP_BUFF_ERR */
	FSL_RETURN_BAD_KEY_PARITY_S,	/* SAH_CHA_DES_KEY_ERR */
	FSL_RETURN_INTERNAL_ERROR_S,	/* SAH_CHA_RES */
};

#ifdef DIAG_DRV_STATUS

char sah_return_text[FSL_INVALID_RETURN][MAX_RETURN_STRING_LEN] = {
	"No error",		/* FSL_RETURN_OK_S */
	"Error",		/* FSL_RETURN_ERROR_S */
	"No resource",		/* FSL_RETURN_NO_RESOURCE_S */
	"Bad algorithm",	/* FSL_RETURN_BAD_ALGORITHM_S */
	"Bad mode",		/* FSL_RETURN_BAD_MODE_S */
	"Bad flag",		/* FSL_RETURN_BAD_FLAG_S */
	"Bad key length",	/* FSL_RETURN_BAD_KEY_LENGTH_S */
	"Bad key parity",	/* FSL_RETURN_BAD_KEY_PARITY_S */
	"Bad data length",	/* FSL_RETURN_BAD_DATA_LENGTH_S */
	"Authentication failed",	/* FSL_RETURN_AUTH_FAILED_S */
	"Memory error",		/* FSL_RETURN_MEMORY_ERROR_S */
	"Internal error",	/* FSL_RETURN_INTERNAL_ERROR_S */
	"unknown value",	/* default */
};

#endif				/* DIAG_DRV_STATUS */

/*!
 * This lock must be held while performing any queuing or unqueuing functions,
 * including reading the first pointer on the queue.  It also protects reading
 * and writing the Sahara DAR register.  It must be held during a read-write
 * operation on the DAR so that the 'test-and-set' is atomic.
 */
os_lock_t desc_queue_lock;

/*! This is the main queue for the driver. This is shared between all threads
 * and is not protected by mutexes since the kernel is non-preemptable. */
sah_Queue *main_queue = NULL;

/* Internal Prototypes */
sah_Head_Desc *sah_Find_With_State(sah_Queue_Status state);

#ifdef DIAG_DRV_STATUS
void sah_Log_Error(uint32_t descriptor, uint32_t error, uint32_t fault_address);
#endif

extern wait_queue_head_t *int_queue;

/*!
 * This function initialises the Queue Manager
 *
 * @brief     Initialise the Queue Manager
 *
 * @return   FSL_RETURN_OK_S on success; FSL_RETURN_MEMORY_ERROR_S if not
 */
fsl_shw_return_t sah_Queue_Manager_Init(void)
{
	fsl_shw_return_t ret_val = FSL_RETURN_OK_S;

	desc_queue_lock = os_lock_alloc_init();

	if (main_queue == NULL) {
		/* Construct the main queue. */
		main_queue = sah_Queue_Construct();

		if (main_queue == NULL) {
			ret_val = FSL_RETURN_MEMORY_ERROR_S;
		}
	} else {
#ifdef DIAG_DRV_QUEUE
		LOG_KDIAG
		    ("Trying to initialise the queue manager more than once.");
#endif
	}

	return ret_val;
}

/*!
 * This function closes the Queue Manager
 *
 * @brief     Close the Queue Manager
 *
 * @return   void
 */
void sah_Queue_Manager_Close(void)
{
#ifdef DIAG_DRV_QUEUE
	if (main_queue && main_queue->count != 0) {
		LOG_KDIAG
		    ("Trying to close the main queue when it is not empty.");
	}
#endif

	if (main_queue) {
		/* There is no error checking here because there is no way to handle
		   it. */
		sah_Queue_Destroy(main_queue);
		main_queue = NULL;
	}
}

/*!
 * Count the number of entries on the Queue Manager's queue
 *
 * @param ignore_state  If non-zero, the @a state parameter is ignored.
 *                      If zero, only entries matching @a state are counted.
 * @param state         State of entry to match for counting.
 *
 * @return        Number of entries which matched criteria
 */
int sah_Queue_Manager_Count_Entries(int ignore_state, sah_Queue_Status state)
{
	int count = 0;
	sah_Head_Desc *current_entry;

	/* Start at the head */
	current_entry = main_queue->head;
	while (current_entry != NULL) {
		if (ignore_state || (current_entry->status == state)) {
			count++;
		}
		/* Jump to the next entry. */
		current_entry = current_entry->next;
	}

	return count;
}

/*!
 * This function removes an entry from the Queue Manager's queue. The entry to
 * be removed can be anywhere in the queue.
 *
 * @brief     Remove an entry from the Queue Manager's queue.
 *
 * @param    entry   A pointer to a sah_Head_Desc to remove from the Queue
 *                   Manager's queue.
 *
 * @pre   The #desc_queue_lock must be held before calling this function.
 *
 * @return   void
 */
void sah_Queue_Manager_Remove_Entry(sah_Head_Desc * entry)
{
	if (entry == NULL) {
#ifdef DIAG_DRV_QUEUE
		LOG_KDIAG("NULL pointer input.");
#endif
	} else {
		sah_Queue_Remove_Any_Entry(main_queue, entry);
	}
}

/*!
 * This function appends an entry to the Queue Managers queue. It primes SAHARA
 * if this entry is the first PENDING entry in the Queue Manager's Queue.
 *
 * @brief     Appends an entry to the Queue Manager's queue.
 *
 * @param    entry   A pointer to a sah_Head_Desc to append to the Queue
 *                   Manager's queue.
 *
 * @pre   The #desc_queue_lock may not may be held when calling this function.
 *
 * @return   void
 */
void sah_Queue_Manager_Append_Entry(sah_Head_Desc * entry)
{
	sah_Head_Desc *current_entry;
	os_lock_context_t int_flags;

#ifdef DIAG_DRV_QUEUE
	if (entry == NULL) {
		LOG_KDIAG("NULL pointer input.");
	}
#endif
	entry->status = SAH_STATE_PENDING;
	os_lock_save_context(desc_queue_lock, int_flags);
	sah_Queue_Append_Entry(main_queue, entry);

	/* Prime SAHARA if the operation that was just appended is the only PENDING
	 * operation in the queue.
	 */
	current_entry = sah_Find_With_State(SAH_STATE_PENDING);
	if (current_entry != NULL) {
		if (current_entry == entry) {
			sah_Queue_Manager_Prime(entry);
		}
	}

	os_unlock_restore_context(desc_queue_lock, int_flags);
}

/*!
 * This function marks all entries in the Queue Manager's queue with state
 * SAH_STATE_RESET.
 *
 * @brief     Mark all entries with state SAH_STATE_RESET
 *
 * @return   void
 *
 * @note This feature needs re-visiting
 */
void sah_Queue_Manager_Reset_Entries(void)
{
	sah_Head_Desc *current_entry = NULL;

	/* Start at the head */
	current_entry = main_queue->head;

	while (current_entry != NULL) {
		/* Set the state. */
		current_entry->status = SAH_STATE_RESET;
		/* Jump to the next entry. */
		current_entry = current_entry->next;
	}
}

/*!
 * This function primes SAHARA for the first time or after the queue becomes
 * empty. Queue lock must have been set by the caller of this routine.
 *
 * @brief    Prime SAHARA.
 *
 * @param    entry   A pointer to a sah_Head_Desc to Prime SAHARA with.
 *
 * @return   void
 */
void sah_Queue_Manager_Prime(sah_Head_Desc * entry)
{
#ifdef DIAG_DRV_QUEUE
	LOG_KDIAG("Priming SAHARA");
	if (entry == NULL) {
		LOG_KDIAG("Trying to prime SAHARA with a NULL entry pointer.");
	}
#endif

#ifndef SUBMIT_MULTIPLE_DARS
	/* BUG FIX: state machine can transition from Done1 Busy2 directly
	 * to Idle. To fix that problem, only one DAR is being allowed on
	 * SAHARA at a time */
	if (sah_Find_With_State(SAH_STATE_ON_SAHARA) != NULL) {
		return;
	}
#endif

#ifdef SAHARA_POWER_MANAGEMENT
	/* check that dynamic power management is not asserted */
	if (!sah_dpm_flag) {
#endif
		/* Make sure nothing is in the DAR */
		if (sah_HW_Read_DAR() == 0) {
#if defined(DIAG_DRV_IF)
			sah_Dump_Chain(&entry->desc, entry->desc.dma_addr);
#endif				/* DIAG_DRV_IF */

			sah_HW_Write_DAR((entry->desc.dma_addr));
			entry->status = SAH_STATE_ON_SAHARA;
		}
#ifdef DIAG_DRV_QUEUE
		else {
			LOG_KDIAG("DAR should be empty when Priming SAHARA");
		}
#endif
#ifdef SAHARA_POWER_MANAGEMENT
	}
#endif
}

#ifndef SAHARA_POLL_MODE

/*!
 * Reset SAHARA, then load the next descriptor on it, if one exists
 */
void sah_reset_sahara_request(void)
{
	sah_Head_Desc *desc;
	os_lock_context_t lock_flags;

#ifdef DIAG_DRV_STATUS
	LOG_KDIAG("Sahara required reset from tasklet, replace chip");
#endif
	sah_HW_Reset();

	/* Now stick in a waiting request */
	os_lock_save_context(desc_queue_lock, lock_flags);
	if ((desc = sah_Find_With_State(SAH_STATE_PENDING))) {
		sah_Queue_Manager_Prime(desc);
	}
	os_unlock_restore_context(desc_queue_lock, lock_flags);
}

/*!
 * Post-process a descriptor chain after the hardware has finished with it.
 *
 * The status of the descriptor could also be checked.  (for FATAL or IGNORED).
 *
 * @param     desc_head   The finished chain
 * @param     error       A boolean to mark whether hardware reported error
 *
 * @pre   The #desc_queue_lock may not be held when calling this function.
 */
void sah_process_finished_request(sah_Head_Desc * desc_head, unsigned error)
{
	os_lock_context_t lock_flags;

	if (!error) {
		desc_head->result = FSL_RETURN_OK_S;
	} else if (desc_head->error_status == -1) {
		/* Disaster!  Sahara has faulted */
		desc_head->result = FSL_RETURN_ERROR_S;
	} else {
		/* translate from SAHARA error status to fsl_shw return values */
		desc_head->result =
		    sah_convert_error_status(desc_head->error_status);
#ifdef DIAG_DRV_STATUS
		sah_Log_Error(desc_head->current_dar, desc_head->error_status,
			      desc_head->fault_address);
#endif
	}

	/* Show that the request has been processd */
	desc_head->status = error ? SAH_STATE_FAILED : SAH_STATE_COMPLETE;

	if (desc_head->uco_flags & FSL_UCO_BLOCKING_MODE) {

		/* Wake up all processes on Sahara queue */
		wake_up_interruptible(int_queue);

	} else {
		os_lock_save_context(desc_queue_lock, lock_flags);
		sah_Queue_Append_Entry(&desc_head->user_info->result_pool,
				       desc_head);
		os_unlock_restore_context(desc_queue_lock, lock_flags);

		/* perform callback */
		if (desc_head->uco_flags & FSL_UCO_CALLBACK_MODE) {
			desc_head->user_info->callback(desc_head->user_info);
		}
	}
}				/* sah_process_finished_request */

/*! Called from bottom half.
 *
 * @pre   The #desc_queue_lock may not be held when calling this function.
 */
void sah_postprocess_queue(unsigned long reset_flag)
{

	/* if SAHARA needs to be reset, do it here. This starts a descriptor chain
	 * if one is ready also */
	if (reset_flag) {
		sah_reset_sahara_request();
	}

	/* now handle the descriptor chain(s) that has/have completed */
	do {
		sah_Head_Desc *first_entry;
		os_lock_context_t lock_flags;

		os_lock_save_context(desc_queue_lock, lock_flags);

		first_entry = main_queue->head;
		if ((first_entry != NULL) &&
		    (first_entry->status == SAH_STATE_OFF_SAHARA)) {
			sah_Queue_Remove_Entry(main_queue);
			os_unlock_restore_context(desc_queue_lock, lock_flags);

			sah_process_finished_request(first_entry,
						     (first_entry->
						      error_status != 0));
		} else {
			os_unlock_restore_context(desc_queue_lock, lock_flags);
			break;
		}
	} while (1);

	return;
}

#endif				/* ifndef SAHARA_POLL_MODE */

/*!
 * This is a helper function for Queue Manager. This function finds the first
 * entry in the Queue Manager's queue whose state matches the given input
 * state. This function starts at the head of the queue and works towards the
 * tail. If a matching entry was found, the address of the entry is returned.
 *
 * @brief     Handle the IDLE state.
 *
 * @param    state   A sah_Queue_Status value.
 *
 * @pre   The #desc_queue_lock must be held before calling this function.
 *
 * @return   A pointer to a sah_Head_Desc that matches the given state.
 * @return   NULL otherwise.
 */
sah_Head_Desc *sah_Find_With_State(sah_Queue_Status state)
{
	sah_Head_Desc *current_entry = NULL;
	sah_Head_Desc *ret_val = NULL;
	int done_looping = FALSE;

	/* Start at the head */
	current_entry = main_queue->head;

	while ((current_entry != NULL) && (done_looping == FALSE)) {
		if (current_entry->status == state) {
			done_looping = TRUE;
			ret_val = current_entry;
		}
		/* Jump to the next entry. */
		current_entry = current_entry->next;
	}

	return ret_val;
}				/* sah_postprocess_queue */

/*!
 * Process the value from the Sahara error status register and convert it into
 * an FSL SHW API error code.
 *
 * Warning, this routine must only be called if an error exists.
 *
 * @param error_status   The value from the error status register.
 *
 * @return    A return code of type #fsl_shw_return_t.
 */
fsl_shw_return_t sah_convert_error_status(uint32_t error_status)
{
	fsl_shw_return_t ret = FSL_RETURN_ERROR_S;	/* catchall */
	uint8_t error_source;
	uint8_t DMA_error_status;
	uint8_t DMA_error_size;

	/* get the error source from the error status register */
	error_source = error_status & SAH_ERROR_MASK;

	/* array size is maximum allowed by mask, so no boundary checking is
	 * needed here */
	ret = sah_Execute_Error_Array[error_source];

	/* is this one that needs additional fields checked to determine the
	 * error condition? */
	if (ret == SHA_ERROR_STATUS_CONTINUE) {
		/* check the DMA fields */
		if (error_source == SAH_ERR_DMA) {
			/* get the DMA transfer error size. If this indicates that no
			 * error was detected, something is seriously wrong */
			DMA_error_size =
			    (error_status >> 9) & SAH_DMA_ERR_SIZE_MASK;
			if (DMA_error_size == SAH_DMA_NO_ERR) {
				ret = FSL_RETURN_INTERNAL_ERROR_S;
			} else {
				/* get DMA error status */
				DMA_error_status = (error_status >> 12) &
				    SAH_DMA_ERR_STATUS_MASK;

				/* the DMA error bits cover all the even numbers. By dividing
				 * by 2 it can be used as an index into the error array */
				ret =
				    sah_DMA_Error_Status_Array[DMA_error_status
							       >> 1];
			}
		} else {	/* not SAH_ERR_DMA, so must be SAH_ERR_CHA */
			uint16_t CHA_error_status;
			uint8_t CHA_error_source;

			/* get CHA Error Source. If this indicates that no error was
			 * detected, something is seriously wrong */
			CHA_error_source =
			    (error_status >> 28) & SAH_CHA_ERR_SOURCE_MASK;
			if (CHA_error_source == SAH_CHA_NO_ERROR) {
				ret = FSL_RETURN_INTERNAL_ERROR_S;
			} else {
				uint32_t mask = 1;
				uint32_t count = 0;

				/* get CHA Error Status */
				CHA_error_status = (error_status >> 16) &
				    SAH_CHA_ERR_STATUS_MASK;

				/* If more than one bit is set (which shouldn't happen), only
				 * the first will be captured */
				if (CHA_error_status != 0) {
					count = 1;
					while (CHA_error_status != mask) {
						++count;
						mask <<= 1;
					}
				}

				ret = sah_CHA_Error_Status_Array[count];
			}
		}
	}

	return ret;
}

fsl_shw_return_t sah_convert_op_status(uint32_t op_status)
{
	unsigned op_source = (op_status >> 28) & 0x7;
	unsigned op_detail = op_status & 0x3f;
	fsl_shw_return_t ret = FSL_RETURN_ERROR_S;

	switch (op_source) {
	case 1:		/* SKHA */
		/* Can't this have "ICV" error from CCM ?? */
		break;
	case 2:		/* MDHA */
		if (op_detail == 1) {
			ret = FSL_RETURN_AUTH_FAILED_S;
		}
		break;
	case 3:		/* RNGA */
		/* Self-test and Compare errors... what to do? */
		break;
	case 4:		/* PKHA */
		switch (op_detail) {
		case 0x01:
			ret = FSL_RETURN_PRIME_S;
			break;
		case 0x02:
			ret = FSL_RETURN_NOT_PRIME_S;
			break;
		case 0x04:
			ret = FSL_RETURN_POINT_AT_INFINITY_S;
			break;
		case 0x08:
			ret = FSL_RETURN_POINT_NOT_AT_INFINITY_S;
			break;
		case 0x10:
			ret = FSL_RETURN_GCD_IS_ONE_S;
			break;
		case 0x20:
			ret = FSL_RETURN_GCD_IS_NOT_ONE_S;
			break;
		default:
			break;
		}
		break;
	default:
		break;
	}
	return ret;
}

#ifdef DIAG_DRV_STATUS

/*!
 * This function logs the diagnostic information for the given error and
 * descriptor address.  Only used for diagnostic purposes.
 *
 * @brief    (debug only) Log a description of hardware-detected error.
 *
 * @param    descriptor    The descriptor address that caused the error
 * @param    error         The SAHARA error code
 * @param    fault_address Value from the Fault address register
 *
 * @return   void
 */
void sah_Log_Error(uint32_t descriptor, uint32_t error, uint32_t fault_address)
{
	char *source_text;	/* verbose error source from register */
	char *address;		/* string buffer for descriptor address */
	char *error_log;	/* the complete logging message */
	char *cha_log = NULL;	/* string buffer for descriptor address */
	char *dma_log = NULL;	/* string buffer for descriptor address */

	uint16_t cha_error = 0;
	uint16_t dma_error = 0;

	uint8_t error_source;
	sah_Execute_Error return_code;

	/* log error code and descriptor address */
	error_source = error & SAH_ERROR_MASK;
	return_code = sah_Execute_Error_Array[error_source];

	source_text = os_alloc_memory(64, GFP_KERNEL);

	switch (error_source) {
	case SAH_ERR_HEADER:
		sprintf(source_text, "%s", "Header is not valid");
		break;

	case SAH_ERR_DESC_LENGTH:
		sprintf(source_text, "%s",
			"Descriptor length not equal to sum of link lengths");
		break;

	case SAH_ERR_DESC_POINTER:
		sprintf(source_text, "%s", "Length or pointer "
			"field is zero while the other is non-zero");
		break;

	case SAH_ERR_LINK_LENGTH:
		/* note that the Sahara Block Guide 2.7 has an invalid explaination
		 * of this. It only happens when a link length is zero */
		sprintf(source_text, "%s", "A data length is a link is zero");
		break;

	case SAH_ERR_LINK_POINTER:
		sprintf(source_text, "%s",
			"The data pointer in a link is zero");
		break;

	case SAH_ERR_INPUT_BUFFER:
		sprintf(source_text, "%s", "Input Buffer reported an overflow");
		break;

	case SAH_ERR_OUTPUT_BUFFER:
		sprintf(source_text, "%s",
			"Output Buffer reported an underflow");
		break;

	case SAH_ERR_OUTPUT_BUFFER_STARVATION:
		sprintf(source_text, "%s", "Incorrect data in output "
			"buffer after CHA has signalled 'done'");
		break;

	case SAH_ERR_INTERNAL_STATE:
		sprintf(source_text, "%s", "Internal Hardware Failure");
		break;

	case SAH_ERR_GENERAL_DESCRIPTOR:
		sprintf(source_text, "%s",
			"Current Descriptor was not legal, but cause is unknown");
		break;

	case SAH_ERR_RESERVED_FIELDS:
		sprintf(source_text, "%s",
			"Reserved pointer field is non-zero");
		break;

	case SAH_ERR_DESCRIPTOR_ADDRESS:
		sprintf(source_text, "%s",
			"Descriptor address not word aligned");
		break;

	case SAH_ERR_LINK_ADDRESS:
		sprintf(source_text, "%s", "Link address not word aligned");
		break;

	case SAH_ERR_CHA:
		sprintf(source_text, "%s", "CHA Error");
		{
			char *cha_module = os_alloc_memory(5, GFP_KERNEL);
			char *cha_text = os_alloc_memory(45, GFP_KERNEL);

			cha_error = (error >> 28) & SAH_CHA_ERR_SOURCE_MASK;

			switch (cha_error) {
			case SAH_CHA_SKHA_ERROR:
				sprintf(cha_module, "%s", "SKHA");
				break;

			case SAH_CHA_MDHA_ERROR:
				sprintf(cha_module, "%s", "MDHA");
				break;

			case SAH_CHA_RNG_ERROR:
				sprintf(cha_module, "%s", "RNG ");
				break;

			case SAH_CHA_PKHA_ERROR:
				sprintf(cha_module, "%s", "PKHA");
				break;

			case SAH_CHA_NO_ERROR:
				/* can't happen */
				/* no break */
			default:
				sprintf(cha_module, "%s", "????");
				break;
			}

			cha_error = (error >> 16) & SAH_CHA_ERR_STATUS_MASK;

			/* Log CHA Error Status */
			switch (cha_error) {
			case SAH_CHA_IP_BUF:
				sprintf(cha_text, "%s",
					"Non-empty input buffer when done");
				break;

			case SAH_CHA_ADD_ERR:
				sprintf(cha_text, "%s", "Illegal address");
				break;

			case SAH_CHA_MODE_ERR:
				sprintf(cha_text, "%s", "Illegal mode");
				break;

			case SAH_CHA_DATA_SIZE_ERR:
				sprintf(cha_text, "%s", "Illegal data size");
				break;

			case SAH_CHA_KEY_SIZE_ERR:
				sprintf(cha_text, "%s", "Illegal key size");
				break;

			case SAH_CHA_PROC_ERR:
				sprintf(cha_text, "%s",
					"Mode/Context/Key written during processing");
				break;

			case SAH_CHA_CTX_READ_ERR:
				sprintf(cha_text, "%s",
					"Context read during processing");
				break;

			case SAH_CHA_INTERNAL_HW_ERR:
				sprintf(cha_text, "%s", "Internal hardware");
				break;

			case SAH_CHA_IP_BUFF_ERR:
				sprintf(cha_text, "%s",
					"Input buffer not enabled or underflow");
				break;

			case SAH_CHA_OP_BUFF_ERR:
				sprintf(cha_text, "%s",
					"Output buffer not enabled or overflow");
				break;

			case SAH_CHA_DES_KEY_ERR:
				sprintf(cha_text, "%s", "DES key parity error");
				break;

			case SAH_CHA_RES:
				sprintf(cha_text, "%s", "Reserved");
				break;

			case SAH_CHA_NO_ERR:
				/* can't happen */
				/* no break */
			default:
				sprintf(cha_text, "%s", "Unknown error");
				break;
			}

			cha_log = os_alloc_memory(90, GFP_KERNEL);
			sprintf(cha_log,
				"  Module %s encountered the error: %s.",
				cha_module, cha_text);

			os_free_memory(cha_module);
			os_free_memory(cha_text);

			{
				uint32_t mask = 1;
				uint32_t count = 0;

				if (cha_error != 0) {
					count = 1;
					while (cha_error != mask) {
						++count;
						mask <<= 1;
					}
				}

				return_code = sah_CHA_Error_Status_Array[count];
			}
			cha_error = 1;
		}
		break;

	case SAH_ERR_DMA:
		sprintf(source_text, "%s", "DMA Error");
		{
			char *dma_direction = os_alloc_memory(6, GFP_KERNEL);
			char *dma_size = os_alloc_memory(14, GFP_KERNEL);
			char *dma_text = os_alloc_memory(250, GFP_KERNEL);

			if ((dma_direction == NULL) || (dma_size == NULL) ||
			    (dma_text == NULL)) {
				LOG_KDIAG
				    ("No memory allocated for DMA debug messages\n");
			}

			/* log DMA error direction */
			sprintf(dma_direction, "%s",
				(((error >> 8) & SAH_DMA_ERR_DIR_MASK) == 1) ?
				"read" : "write");

			/* log the size of the DMA transfer error */
			dma_error = (error >> 9) & SAH_DMA_ERR_SIZE_MASK;
			switch (dma_error) {
			case SAH_DMA_SIZE_BYTE:
				sprintf(dma_size, "%s", "byte");
				break;

			case SAH_DMA_SIZE_HALF_WORD:
				sprintf(dma_size, "%s", "half-word");
				break;

			case SAH_DMA_SIZE_WORD:
				sprintf(dma_size, "%s", "word");
				break;

			case SAH_DMA_SIZE_RES:
				sprintf(dma_size, "%s", "reserved size");
				break;

			default:
				sprintf(dma_size, "%s", "unknown size");
				break;
			}

			/* log DMA error status */
			dma_error = (error >> 12) & SAH_DMA_ERR_STATUS_MASK;
			switch (dma_error) {
			case SAH_DMA_NO_ERR:
				sprintf(dma_text, "%s", "No DMA Error Code");
				break;

			case SAH_DMA_AHB_ERR:
				sprintf(dma_text, "%s",
					"AHB terminated a bus cycle with an error");
				break;

			case SAH_DMA_IP_ERR:
				sprintf(dma_text, "%s",
					"Internal IP bus cycle was terminated with an "
					"error termination. This would likely be "
					"caused by a descriptor length being too "
					"large, and thus accessing an illegal "
					"internal address. Verify the length field "
					"of the current descriptor");
				break;

			case SAH_DMA_PARITY_ERR:
				sprintf(dma_text, "%s",
					"Parity error detected on DMA command from "
					"Descriptor Decoder. Cause is likely to be "
					"internal hardware fault");
				break;

			case SAH_DMA_BOUNDRY_ERR:
				sprintf(dma_text, "%s",
					"DMA was requested to cross a 256 byte "
					"internal address boundary. Cause is likely a "
					"descriptor length being too large, thus "
					"accessing two different internal hardware "
					"blocks");
				break;

			case SAH_DMA_BUSY_ERR:
				sprintf(dma_text, "%s",
					"Descriptor Decoder has made a DMA request "
					"while the DMA controller is busy. Cause is "
					"likely due to hardware fault");
				break;

			case SAH_DMA_RESERVED_ERR:
				sprintf(dma_text, "%s", "Reserved");
				break;

			case SAH_DMA_INT_ERR:
				sprintf(dma_text, "%s",
					"Internal DMA hardware error detected. The "
					"DMA controller has detected an internal "
					"condition which should never occur");
				break;

			default:
				sprintf(dma_text, "%s",
					"Unknown DMA Error Status Code");
				break;
			}

			return_code =
			    sah_DMA_Error_Status_Array[dma_error >> 1];
			dma_error = 1;

			dma_log = os_alloc_memory(320, GFP_KERNEL);
			sprintf(dma_log,
				"  Occurred during a %s operation of a %s transfer: %s.",
				dma_direction, dma_size, dma_text);

			os_free_memory(dma_direction);
			os_free_memory(dma_size);
			os_free_memory(dma_text);
		}
		break;

	case SAH_ERR_NONE:
	default:
		sprintf(source_text, "%s", "Unknown Error Code");
		break;
	}

	address = os_alloc_memory(35, GFP_KERNEL);

	/* convert error & descriptor address to strings */
	if (dma_error) {
		sprintf(address, "Fault address is 0x%08x", fault_address);
	} else {
		sprintf(address, "Descriptor bus address is 0x%08x",
			descriptor);
	}

	if (return_code > FSL_INVALID_RETURN) {
		return_code = FSL_INVALID_RETURN;
	}

	error_log = os_alloc_memory(250, GFP_KERNEL);

	/* construct final log message */
	sprintf(error_log, "Error source = 0x%08x. Return = %s. %s. %s.",
		error, sah_return_text[return_code], address, source_text);

	os_free_memory(source_text);
	os_free_memory(address);

	/* log standard messages */
	LOG_KDIAG(error_log);
	os_free_memory(error_log);

	/* add additional information if available */
	if (cha_error) {
		LOG_KDIAG(cha_log);
		os_free_memory(cha_log);
	}

	if (dma_error) {
		LOG_KDIAG(dma_log);
		os_free_memory(dma_log);
	}

	return;
}				/* sah_Log_Error */

#endif				/* DIAG_DRV_STATUS */

/* End of sah_queue_manager.c */
