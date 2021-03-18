/*
 * Copyright 2018 NXP
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
 */
#ifndef VEHICLE_CORE_H
#define VEHICLE_CORE_H

struct vehicle_property_set {
	u32 prop;
	u32 area_id;
	u32 value;
};

struct vehicle_power_req {
	u32 prop;
	u32 state;
	u32 param;
};

struct hw_prop_ops {
	/* send the control commands to hw */
	void (*set_control_commands)(u32 prop, u32 area, u32 value);
};

#ifdef CONFIG_VEHICLE_DRIVER_OREO
#define HVAC_FAN_SPEED 306185472
#define HVAC_FAN_DIRECTION 306185473
#define HVAC_AUTO_ON 304088330
#define HVAC_AC_ON 304088325
#define HVAC_RECIRC_ON 304088328
#define HVAC_DEFROSTER 320865540
#define HVAC_TEMPERATURE_SET 308282627
#define HVAC_POWER_ON 304088336
#else
#define HVAC_FAN_SPEED 356517120
#define HVAC_FAN_DIRECTION 356517121
#define HVAC_AUTO_ON 354419978
#define HVAC_AC_ON 354419973
#define HVAC_RECIRC_ON 354419976
#define HVAC_DEFROSTER 320865540
#define HVAC_TEMPERATURE_SET 358614275
#define HVAC_POWER_ON 354419984
#endif

#define TURN_SIGNAL_STATE 289408008
#define GEAR_SELECTION 289408000

#define AP_POWER_STATE_REQ 289475072
#define AP_POWER_STATE_REPORT 289475073

#define VEHICLE_GEAR_DRIVE_CLIENT 8
#define VEHICLE_GEAR_PARK_CLIENT 4
#define VEHICLE_GEAR_REVERSE_CLIENT 2

/*vehicle_event_type: stateType in command VSTATE*/
enum vehicle_event_type {
	VEHICLE_AC = 0,
	VEHICLE_AUTO_ON,
	VEHICLE_AC_TEMP,
	VEHICLE_FAN_SPEED,
	VEHICLE_FAN_DIRECTION,
	VEHICLE_RECIRC_ON,
	VEHICLE_HEATER,
	VEHICLE_DEFROST,
	VEHICLE_HVAC_POWER_ON,
	VEHICLE_MUTE,
	VEHICLE_VOLUME,
	VEHICLE_DOOR,
	VEHICLE_RVC,
	VEHICLE_LIGHT,
	VEHICLE_GEAR,
	VEHICLE_TURN_SIGNAL,
	VEHICLE_POWER_STATE_REQ,
};

/*vehicle_event_gear: stateValue of type VEHICLE_GEAR*/
enum vehicle_event_gear {
	VEHICLE_GEAR_NONE = 0,
	VEHICLE_GEAR_PARKING,
	VEHICLE_GEAR_REVERSE,
	VEHICLE_GEAR_NEUTRAL,
	VEHICLE_GEAR_DRIVE,
	VEHICLE_GEAR_FIRST,
	VEHICLE_GEAR_SECOND,
	VEHICLE_GEAR_SPORT,
	VEHICLE_GEAR_MANUAL_1,
	VEHICLE_GEAR_MANUAL_2,
	VEHICLE_GEAR_MANUAL_3,
	VEHICLE_GEAR_MANUAL_4,
	VEHICLE_GEAR_MANUAL_5,
	VEHICLE_GEAR_MANUAL_6,
};

#endif
