/*
 * Copyright 2007-2008 Freescale Semiconductor, Inc. All Rights Reserved.
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
 * hw_events.h
 * include the headset/cvbs interrupt detect
 */

#ifndef HW_EVENT_H
#define HW_EVENT_H

#define HW_EVENT_GROUP		2
#define HWE_DEF_PRIORITY	1
#define HWE_HIGH_PRIORITY	0

typedef enum {

	HWE_PHONEJACK_PLUG = 0,
	HWE_BAT_CHARGER_PLUG,
	HWE_BAT_CHARGER_OVERVOLTAGE,
	HWE_BAT_BATTERY_LOW,
	HWE_BAT_POWER_FAILED,
	HWE_BAT_CHARGER_FULL,
	HWE_POWER_KEY,
} HW_EVENT_T;

typedef enum {

	PJT_NONE = 0,
	PJT_CVBS,
	PJT_HEADSET,
} PHONEJACK_TYPE;

typedef enum {

	PWRK_UNPRESS = 0,
	PWRK_PRESS,
} POWERKEY_TYPE;

typedef enum {

	UNPLUG = 0,
	PLUGGED,
} PLUG_TYPE;

struct mxc_hw_event {
	unsigned int event;
	int args;
};

#ifdef __KERNEL__
extern int hw_event_send(int priority, struct mxc_hw_event *new_event);
#endif

#endif				/* HW_EVENT_H */
