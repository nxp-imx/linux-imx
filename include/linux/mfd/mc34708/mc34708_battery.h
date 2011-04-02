/*
 * Copyright (C) 2011 Freescale Semiconductor, Inc. All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
 */

#ifndef _MC34708_BC_H
#define _MC34708_BC_H

#include <linux/types.h>

#define RIPLIEY_BC_MAX_RESTART_CYCLES 100

#define RIPLIEY_BC_LIION_CHARGING_VOLTAGE  4200
#define RIPLIEY_BC_ALKALINE_NIMH_CHARGING_VOLTAGE 1750

/* brief Defines battery charger states. */
typedef enum _ddi_bc_State {
	/* brief TBD */
	RIPLIEY_BC_STATE_UNINITIALIZED = 0,
	/* brief TBD */
	RIPLIEY_BC_STATE_BROKEN = 1,
	/* brief TBD */
	RIPLIEY_BC_STATE_DISABLED = 2,
	/* brief TBD */
	RIPLIEY_BC_STATE_WAITING_TO_CHARGE = 3,
	/* brief TBD */
	RIPLIEY_BC_STATE_CONDITIONING = 4,
	/* brief TBD */
	RIPLIEY_BC_STATE_CHARGING = 5,
	/* brief TBD */
	RIPLIEY_BC_STATE_TOPPING_OFF = 6,
	/* brief TBD */
	RIPLIEY_BC_STATE_DCDC_MODE_WAITING_TO_CHARGE = 7,

} ddi_bc_State_t;

typedef enum _ddi_bc_BrokenReason {
	/* brief TBD */
	RIPLIEY_BC_BROKEN_UNINITIALIZED = 0,
	/* brief TBD */
	RIPLIEY_BC_BROKEN_CHARGING_TIMEOUT = 1,
	/* brief TBD */
	RIPLIEY_BC_BROKEN_FORCED_BY_APPLICATION = 2,
	/* brief TBD */
	RIPLIEY_BC_BROKEN_EXTERNAL_BATTERY_VOLTAGE_DETECTED = 3,
	/* brief TBD */
	RIPLIEY_BC_BROKEN_NO_BATTERY_DETECTED = 4,

} ddi_bc_BrokenReason_t;

struct mc34708_charger_setting_point {
	u32 microVolt;
	u32 microAmp;
};

/* brief Defines the battery charger configuration. */
typedef struct mc34708_charger_config {

	u32 batteryTempLow;
	u32 batteryTempHigh;
	s32 hasTempSensor;

	u32 trickleThreshold;

	u32 vbusThresholdLow;
	u32 vbusThresholdWeak;
	u32 vbusThresholdHigh;

	u32 vauxThresholdLow;
	u32 vauxThresholdWeak;
	u32 vauxThresholdHigh;

	u32 lowBattThreshold;

	u32 toppingOffMicroAmp;

	struct mc34708_charger_setting_point *chargingPoints;
	u32 pointsNumber;

};

/*  Status returned by Battery Charger functions. */

typedef enum _ddi_bc_Status {
	/* brief TBD */
	RIPLIEY_BC_STATUS_SUCCESS = 0,
	/* brief TBD */
	RIPLIEY_BC_STATUS_HARDWARE_DISABLED,
	/* brief TBD */
	RIPLIEY_BC_STATUS_BAD_BATTERY_MODE,
	/* brief TBD */
	RIPLIEY_BC_STATUS_CLOCK_GATE_CLOSED,
	/* brief TBD */
	RIPLIEY_BC_STATUS_NOT_INITIALIZED,
	/* brief TBD */
	RIPLIEY_BC_STATUS_ALREADY_INITIALIZED,
	/* brief TBD */
	RIPLIEY_BC_STATUS_BROKEN,
	/* brief TBD */
	RIPLIEY_BC_STATUS_NOT_BROKEN,
	/* brief TBD */
	RIPLIEY_BC_STATUS_NOT_DISABLED,
	/* brief TBD */
	RIPLIEY_BC_STATUS_BAD_ARGUMENT,
	/* brief TBD */
	RIPLIEY_BC_STATUS_CFG_BAD_BATTERY_TEMP_CHANNEL,
	/* brief TBD */
	RIPLIEY_BC_STATUS_CFG_BAD_CHARGING_VOLTAGE,
} ddi_bc_Status_t;


/*  BCM Event Codes */

/* These are the codes that might be published to PMI Subscribers. */


#define RIPLIEY_BC_EVENT_GROUP (11<<10)

/* brief TBD */
/* todo [PUBS] Add definition(s)... */
typedef enum {
	/* Use the error code group value to make events unique for the EOI */
	/* brief TBD */
	ddi_bc_MinEventCode = RIPLIEY_BC_EVENT_GROUP,
	/* brief TBD */
	ddi_bc_WaitingToChargeCode,
	/* brief TBD */
	ddi_bc_State_ConditioningCode,
	/* brief TBD */
	ddi_bc_State_Topping_OffCode,
	/* brief TBD */
	ddi_bc_State_BrokenCode,
	/* brief TBD */
	ddi_bc_SettingChargeCode,
	/* brief TBD */
	ddi_bc_RaisingDieTempAlarmCode,
	/* brief TBD */
	ddi_bc_DroppingDieTempAlarmCode,

	/* brief TBD */
	ddi_bc_MaxEventCode,
	/* brief TBD */
	ddi_bc_DcdcModeWaitingToChargeCode
} ddi_bc_Event_t;


/* End of file */

#endif				/* _MC34708_BC_H */
