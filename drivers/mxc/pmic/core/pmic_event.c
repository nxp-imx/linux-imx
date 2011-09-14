/*
 * Copyright 2004-2011 Freescale Semiconductor, Inc. All Rights Reserved.
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
 * @file pmic_event.c
 * @brief This file manage all event of PMIC component.
 *
 * It contains event subscription, unsubscription and callback
 * launch methods implemeted.
 *
 * @ingroup PMIC_CORE
 */

/*
 * Includes
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/errno.h>
#include <linux/interrupt.h>
#include <linux/list.h>
#include <linux/mutex.h>
#include <linux/pmic_external.h>
#include <linux/pmic_status.h>
#include "pmic.h"

/*!
 * This structure is used to keep a list of subscribed
 * callbacks for an event.
 */
typedef struct {
	/*!
	 * Keeps a list of subscribed clients to an event.
	 */
	struct list_head list;

	/*!
	 * Callback function with parameter, called when event occurs
	 */
	pmic_event_callback_t callback;
} pmic_event_callback_list_t;

/* Create a mutex to be used to prevent concurrent access to the event list */
static DEFINE_MUTEX(event_mutex);

/* This is a pointer to the event handler array. It defines the currently
 * active set of events and user-defined callback functions.
 */
static struct list_head pmic_events[PMIC_MAX_EVENTS];

/*!
 * This function initializes event list for PMIC event handling.
 *
 */
void pmic_event_list_init(void)
{
	int i;

	for (i = 0; i < PMIC_MAX_EVENTS; i++) {
		INIT_LIST_HEAD(&pmic_events[i]);
	}

	mutex_init(&event_mutex);
	return;
}

/*!
 * This function is used to subscribe on an event.
 *
 * @param	event   the event number to be subscribed
 * @param	callback the callback funtion to be subscribed
 *
 * @return       This function returns 0 on SUCCESS, error on FAILURE.
 */
PMIC_STATUS pmic_event_subscribe(type_event event,
				 pmic_event_callback_t callback)
{
	pmic_event_callback_list_t *new = NULL;

	pr_debug("Event:%d Subscribe\n", event);

	/* Check whether the event & callback are valid? */
	if (event >= PMIC_MAX_EVENTS) {
		pr_debug("Invalid Event:%d\n", event);
		return -EINVAL;
	}
	if (NULL == callback.func) {
		pr_debug("Null or Invalid Callback\n");
		return -EINVAL;
	}

	/* Create a new linked list entry */
	new = kmalloc(sizeof(pmic_event_callback_list_t), GFP_KERNEL);
	if (NULL == new) {
		return -ENOMEM;
	}
	/* Initialize the list node fields */
	new->callback.func = callback.func;
	new->callback.param = callback.param;
	INIT_LIST_HEAD(&new->list);

	/* Obtain the lock to access the list */
	if (mutex_lock_interruptible(&event_mutex)) {
		kfree(new);
		return PMIC_SYSTEM_ERROR_EINTR;
	}

	/* Unmask the requested event */
	if (list_empty(&pmic_events[event])) {
		if (pmic_event_unmask(event) != PMIC_SUCCESS) {
			kfree(new);
			mutex_unlock(&event_mutex);
			return PMIC_ERROR;
		}
	}

	/* Add this entry to the event list */
	list_add_tail(&new->list, &pmic_events[event]);

	/* Release the lock */
	mutex_unlock(&event_mutex);

	return PMIC_SUCCESS;
}

/*!
 * This function is used to unsubscribe on an event.
 *
 * @param	event   the event number to be unsubscribed
 * @param	callback the callback funtion to be unsubscribed
 *
 * @return       This function returns 0 on SUCCESS, error on FAILURE.
 */
PMIC_STATUS pmic_event_unsubscribe(type_event event,
				   pmic_event_callback_t callback)
{
	struct list_head *p;
	struct list_head *n;
	pmic_event_callback_list_t *temp = NULL;
	int ret = PMIC_EVENT_NOT_SUBSCRIBED;

	pr_debug("Event:%d Unsubscribe\n", event);

	/* Check whether the event & callback are valid? */
	if (event >= PMIC_MAX_EVENTS) {
		pr_debug("Invalid Event:%d\n", event);
		return -EINVAL;
	}

	if (NULL == callback.func) {
		pr_debug("Null or Invalid Callback\n");
		return -EINVAL;
	}

	/* Obtain the lock to access the list */
	if (mutex_lock_interruptible(&event_mutex)) {
		return PMIC_SYSTEM_ERROR_EINTR;
	}

	/* Find the entry in the list */
	list_for_each_safe(p, n, &pmic_events[event]) {
		temp = list_entry(p, pmic_event_callback_list_t, list);
		if (temp->callback.func == callback.func
		    && temp->callback.param == callback.param) {
			/* Remove the entry from the list */
			list_del(p);
			kfree(temp);
			ret = PMIC_SUCCESS;
			break;
		}
	}

	/* Unmask the requested event */
	if (list_empty(&pmic_events[event])) {
		if (pmic_event_mask(event) != PMIC_SUCCESS) {
			ret = PMIC_UNSUBSCRIBE_ERROR;
		}
	}

	/* Release the lock */
	mutex_unlock(&event_mutex);

	return ret;
}

/*!
 * This function calls all callback of a specific event.
 *
 * @param	event   the active event number
 *
 * @return 	None
 */
void pmic_event_callback(type_event event)
{
	struct list_head *p;
	pmic_event_callback_list_t *temp = NULL;

	/* Obtain the lock to access the list */
	if (mutex_lock_interruptible(&event_mutex)) {
		return;
	}

	if (list_empty(&pmic_events[event])) {
		pr_debug("PMIC Event:%d detected. No callback subscribed\n",
			 event);
		mutex_unlock(&event_mutex);
		return;
	}

	list_for_each(p, &pmic_events[event]) {
		temp = list_entry(p, pmic_event_callback_list_t, list);
		temp->callback.func(temp->callback.param);
	}

	/* Release the lock */
	mutex_unlock(&event_mutex);

	return;

}

EXPORT_SYMBOL(pmic_event_subscribe);
EXPORT_SYMBOL(pmic_event_unsubscribe);
