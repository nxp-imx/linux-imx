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
* @file sah_queue.c
*
* @brief This file provides a FIFO Queue implementation.
*
*/
/******************************************************************************
*
* CAUTION:
*******************************************************************
*/

/* SAHARA Includes */
#include <sah_queue_manager.h>
#ifdef DIAG_DRV_QUEUE
#include <diagnostic.h>
#endif

/******************************************************************************
* Queue Functions
******************************************************************************/

/*!
*******************************************************************************
* This function constructs a new sah_Queue.
*
* @brief     sah_Queue Constructor
*
* @return   A pointer to a newly allocated sah_Queue.
* @return   NULL if allocation of memory failed.
*/
/******************************************************************************
*
* CAUTION:  This function may sleep in low-memory situations, as it uses
*           kmalloc ( ..., GFP_KERNEL).
******************************************************************************/
sah_Queue *sah_Queue_Construct(void)
{
	sah_Queue *q = (sah_Queue *) os_alloc_memory(sizeof(sah_Queue),
						     GFP_KERNEL);

	if (q != NULL) {
		/* Initialise the queue to an empty state. */
		q->head = NULL;
		q->tail = NULL;
		q->count = 0;
	}
#ifdef DIAG_DRV_QUEUE
	else {
		LOG_KDIAG("kmalloc() failed.");
	}
#endif

	return q;
}

/*!
*******************************************************************************
* This function destroys a sah_Queue.
*
* @brief     sah_Queue Destructor
*
* @param    q    A pointer to a sah_Queue.
*
* @return   void
*/
/******************************************************************************
*
* CAUTION:  This function does not free any queue entries.
*
******************************************************************************/
void sah_Queue_Destroy(sah_Queue * q)
{
#ifdef DIAG_DRV_QUEUE
	if (q == NULL) {
		LOG_KDIAG("Trying to kfree() a NULL pointer.");
	} else {
		if (q->count != 0) {
			LOG_KDIAG
			    ("Trying to destroy a queue that is not empty.");
		}
	}
#endif

	if (q != NULL) {
		os_free_memory(q);
		q = NULL;
	}
}

/*!
*******************************************************************************
* This function appends a sah_Head_Desc to the tail of a sah_Queue.
*
* @brief     Appends a sah_Head_Desc to a sah_Queue.
*
* @param    q       A pointer to a sah_Queue to append to.
* @param    entry   A pointer to a sah_Head_Desc to append.
*
* @pre   The #desc_queue_lock must be held before calling this function.
*
* @return   void
*/
/******************************************************************************
*
* CAUTION: NONE
******************************************************************************/
void sah_Queue_Append_Entry(sah_Queue * q, sah_Head_Desc * entry)
{
	sah_Head_Desc *tail_entry = NULL;

	if ((q == NULL) || (entry == NULL)) {
#ifdef DIAG_DRV_QUEUE
		LOG_KDIAG("Null pointer input.");
#endif
		return;
	}

	if (q->count == 0) {
		/* The queue is empty */
		q->head = entry;
		q->tail = entry;
		entry->next = NULL;
		entry->prev = NULL;
	} else {
		/* The queue is not empty */
		tail_entry = q->tail;
		tail_entry->next = entry;
		entry->next = NULL;
		entry->prev = tail_entry;
		q->tail = entry;
	}
	q->count++;
}

/*!
*******************************************************************************
* This function a removes a sah_Head_Desc from the head of a sah_Queue.
*
* @brief     Removes a sah_Head_Desc from a the head of a sah_Queue.
*
* @param    q    A pointer to a sah_Queue to remove from.
*
* @pre   The #desc_queue_lock must be held before calling this function.
*
* @return   void
*/
/******************************************************************************
*
* CAUTION:  This does not kfree() the entry.
******************************************************************************/
void sah_Queue_Remove_Entry(sah_Queue * q)
{
	sah_Queue_Remove_Any_Entry(q, q->head);
}

/*!
*******************************************************************************
* This function a removes a sah_Head_Desc from anywhere in a sah_Queue.
*
* @brief     Removes a sah_Head_Desc from anywhere in a sah_Queue.
*
* @param    qq      A pointer to a sah_Queue to remove from.
* @param    entry   A pointer to a sah_Head_Desc to remove.
*
* @pre   The #desc_queue_lock must be held before calling this function.
*
* @return   void
*/
/******************************************************************************
*
* CAUTION:  This does not kfree() the entry. Does not check to see if the entry
*           actually belongs to the queue.
******************************************************************************/
void sah_Queue_Remove_Any_Entry(sah_Queue * q, sah_Head_Desc * entry)
{
	sah_Head_Desc *prev_entry = NULL;
	sah_Head_Desc *next_entry = NULL;

	if ((q == NULL) || (entry == NULL)) {
#if defined DIAG_DRV_QUEUE && defined DIAG_DURING_INTERRUPT
		LOG_KDIAG("Null pointer input.");
#endif
		return;
	}

	if (q->count == 1) {
		/* If q is the only entry in the queue. */
		q->tail = NULL;
		q->head = NULL;
		q->count = 0;
	} else if (q->count > 1) {
		/* There are 2 or more entries in the queue. */

#if defined DIAG_DRV_QUEUE && defined DIAG_DURING_INTERRUPT
		if ((entry->next == NULL) && (entry->prev == NULL)) {
			LOG_KDIAG
			    ("Queue is not empty yet both next and prev pointers"
			     " are NULL");
		}
#endif

		if (entry->next == NULL) {
			/* If this is the end of the queue */
			prev_entry = entry->prev;
			prev_entry->next = NULL;
			q->tail = prev_entry;
		} else if (entry->prev == NULL) {
			/* If this is the head of the queue */
			next_entry = entry->next;
			next_entry->prev = NULL;
			q->head = next_entry;
		} else {
			/* If this is somewhere in the middle of the queue */
			prev_entry = entry->prev;
			next_entry = entry->next;
			prev_entry->next = next_entry;
			next_entry->prev = prev_entry;
		}
		q->count--;
	}
	/*
	 * Otherwise we are removing an entry from an empty queue.
	 * Don't do anything in the product code
	 */
#if defined DIAG_DRV_QUEUE && defined DIAG_DURING_INTERRUPT
	else {
		LOG_KDIAG("Trying to remove an entry from an empty queue.");
	}
#endif

	entry->next = NULL;
	entry->prev = NULL;
}

/* End of sah_queue.c */
