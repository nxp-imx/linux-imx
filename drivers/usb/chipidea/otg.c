/*
 * otg.c - ChipIdea USB IP core OTG driver
 *
 * Copyright (C) 2013-2014 Freescale Semiconductor, Inc.
 *
 * Author: Peter Chen
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

/*
 * This file mainly handles otgsc register, it may include OTG operation
 * in the future.
 */

#include <linux/usb/otg.h>
#include <linux/usb/gadget.h>
#include <linux/usb/chipidea.h>
#include <linux/kthread.h>
#include <linux/freezer.h>

#include "ci.h"
#include "bits.h"
#include "otg.h"

/**
 * hw_read_otgsc returns otgsc register bits value.
 * @mask: bitfield mask
 */
 u32 hw_read_otgsc(struct ci_hdrc *ci, u32 mask)
 {
	 return hw_read(ci, OP_OTGSC, mask);
 }

/**
 * hw_write_otgsc updates target bits of OTGSC register.
 * @mask: bitfield mask
 * @data: to be written
 */
void hw_write_otgsc(struct ci_hdrc *ci, u32 mask, u32 data)
{
	hw_write(ci, OP_OTGSC, mask | OTGSC_INT_STATUS_BITS, data);
}

/**
 * ci_otg_role - pick role based on ID pin state
 * @ci: the controller
 */
enum ci_role ci_otg_role(struct ci_hdrc *ci)
{
	enum ci_role role = hw_read_otgsc(ci, OTGSC_ID)
		? CI_ROLE_GADGET
		: CI_ROLE_HOST;

	return role;
}

void ci_handle_vbus_connected(struct ci_hdrc *ci)
{
	u32 otgsc;

	/*
	 * TODO: if the platform does not supply 5v to udc, or use other way
	 * to supply 5v, it needs to use other conditions to call
	 * usb_gadget_vbus_connect.
	 */
	if (!ci->is_otg)
		return;

	otgsc = hw_read(ci, OP_OTGSC, ~0);

	if (otgsc & OTGSC_BSV)
		usb_gadget_vbus_connect(&ci->gadget);
}

void ci_handle_vbus_change(struct ci_hdrc *ci)
{
	if (!ci->is_otg)
		return;

	if (hw_read_otgsc(ci, OTGSC_BSV))
		usb_gadget_vbus_connect(&ci->gadget);
	else
		usb_gadget_vbus_disconnect(&ci->gadget);
}

#define CI_VBUS_STABLE_TIMEOUT_MS 5000
static void ci_handle_id_switch(struct ci_hdrc *ci)
{
	enum ci_role role = ci_otg_role(ci);

	if (role != ci->role) {
		dev_dbg(ci->dev, "switching from %s to %s\n",
			ci_role(ci)->name, ci->roles[role]->name);

		ci_role_stop(ci);
		/* wait vbus lower than OTGSC_BSV */
		hw_wait_reg(ci, OP_OTGSC, OTGSC_BSV, 0,
				CI_VBUS_STABLE_TIMEOUT_MS);
		ci_role_start(ci, role);
	}
}

/* If there is pending otg event */
static inline bool ci_otg_event_is_pending(struct ci_hdrc *ci)
{
	return ci->id_event || ci->b_sess_valid_event;
}

/**
 * ci_otg_event - perform otg (vbus/id) event handle
 * @ci: ci_hdrc struct
 */
static void ci_otg_event(struct ci_hdrc *ci)
{
	if (ci->id_event) {
		ci->id_event = false;
		/* Keep controller active during id switch */
		pm_runtime_get_sync(ci->dev);
		ci_handle_id_switch(ci);
		pm_runtime_put_sync(ci->dev);
	} else if (ci->b_sess_valid_event) {
		ci->b_sess_valid_event = false;
		pm_runtime_get_sync(ci->dev);
		ci_handle_vbus_change(ci);
		pm_runtime_put_sync(ci->dev);
	} else
		dev_dbg(ci->dev, "it should be quit event\n");

	enable_irq(ci->irq);
}

static int ci_otg_thread(void *ptr)
{
	struct ci_hdrc *ci = ptr;

	set_freezable();

	do {
		wait_event_freezable(ci->otg_wait,
				ci_otg_event_is_pending(ci) ||
				kthread_should_stop());
		ci_otg_event(ci);
	} while (!kthread_should_stop());

	dev_warn(ci->dev, "ci_otg_thread quits\n");

	return 0;
}

/**
 * ci_hdrc_otg_init - initialize otg struct
 * ci: the controller
 */
int ci_hdrc_otg_init(struct ci_hdrc *ci)
{
	init_waitqueue_head(&ci->otg_wait);
	ci->otg_task = kthread_run(ci_otg_thread, ci, "ci otg thread");
	if (IS_ERR(ci->otg_task)) {
		dev_err(ci->dev, "error to create otg thread\n");
		return PTR_ERR(ci->otg_task);
	}

	return 0;
}

/**
 * ci_hdrc_otg_destroy - destroy otg struct
 * ci: the controller
 */
void ci_hdrc_otg_destroy(struct ci_hdrc *ci)
{
	kthread_stop(ci->otg_task);
	hw_write_otgsc(ci, OTGSC_INT_EN_BITS | OTGSC_INT_STATUS_BITS,
						OTGSC_INT_STATUS_BITS);
}
