/*
 * Copyright (C) 2012 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

#ifndef _DDI_BC_H
#define _DDI_BC_H

#include <linux/types.h>

#define DDI_BC_MAX_RESTART_CYCLES 100

#define DDI_BC_LIION_CHARGING_VOLTAGE  4200
#define DDI_BC_ALKALINE_NIMH_CHARGING_VOLTAGE 1750

/* brief Defines battery charger states. */
typedef enum _ddi_bc_State {
	/* brief TBD */
	DDI_BC_STATE_UNINITIALIZED = 0,
	/* brief TBD */
	DDI_BC_STATE_BROKEN = 1,
	/* brief TBD */
	DDI_BC_STATE_DISABLED = 2,
	/* brief TBD */
	DDI_BC_STATE_WAITING_TO_CHARGE = 3,
	/* brief TBD */
	DDI_BC_STATE_CONDITIONING = 4,
	/* brief TBD */
	DDI_BC_STATE_CHARGING = 5,
	/* brief TBD */
	DDI_BC_STATE_TOPPING_OFF = 6,
	/* brief TBD */
	DDI_BC_STATE_DCDC_MODE_WAITING_TO_CHARGE = 7,

} ddi_bc_State_t;

typedef enum _ddi_bc_BrokenReason {
	/* brief TBD */
	DDI_BC_BROKEN_UNINITIALIZED = 0,
	/* brief TBD */
	DDI_BC_BROKEN_CHARGING_TIMEOUT = 1,
	/* brief TBD */
	DDI_BC_BROKEN_FORCED_BY_APPLICATION = 2,
	/* brief TBD */
	DDI_BC_BROKEN_EXTERNAL_BATTERY_VOLTAGE_DETECTED = 3,
	/* brief TBD */
	DDI_BC_BROKEN_NO_BATTERY_DETECTED = 4,

} ddi_bc_BrokenReason_t;

/* brief Defines the battery charger configuration. */
typedef struct _ddi_bc_Cfg {
	/* brief Units in milliseconds. */
	/*  */
	/*  This field configures the expected period between calls to */
	/*  ddi_bc_StateMachine. If die temperature monitoring is */
	/*  enabled, then the data sheet recommends the period be around */
	/*  100ms or less. */
	/*  */
	/*  Note that this period defines the minimum time resolution of */
	/*  the battery charger. */

	uint32_t u32StateMachinePeriod;

	/* brief Units in mA/s. */
	/*  */
	/*  This field configures the slope of the current ramp. Any */
	/*  time the battery charger increases its current draw, it will */
	/*  ramp up the current no faster than this rate. */
	/*  */
	/*  Note that the minimum time resolution of the battery charger */
	/*  is the configured period between calls to advance the state */
	/*  machine. Also, the hardware has a minimum current resolution */
	/*  of 10mA. If the given ramp slope cannot be expressed */
	/*  exactly, then the largest expressible smaller slope will be */
	/*  the result. If the actual period between calls to */
	/*  ddi_bc_StateMachine is irregular, the current may ramp faster */
	/*  than indicated. */

	uint16_t u16CurrentRampSlope;

	/* brief Units in millivolts. */
	/*  */
	/*  This field configures the threshold conditioning voltage. If */
	/*  the battery's voltage is below this value, it will be */
	/*  conditioned until its voltage rises above the maximum */
	/*  conditioning voltage.  After that, the battery will be */
	/*  charged normally. */
	/*  */
	/*  Note that the hardware has a minimum resolution of 8mV. If */
	/*  the given voltage cannot be expressed exactly, then the */
	/*  smallest expressible larger value will be used. */

	uint16_t u16ConditioningThresholdVoltage;

	/* brief Units in millivolts. */
	/*  */
	/*  This field configures the maximum conditioning voltage. If */
	/*  the battery charger is conditioning a battery, normal */
	/*  charging begins when the voltage rises above this value. */
	/*  */
	/*  This value should be slightly higher than the threshold */
	/*  conditioning voltage because it is measured while a */
	/*  conditioning current is actually flowing to the battery. */
	/*  With a conditioning current of 0.1C, reasonable values for */
	/*  the threshold and maximum conditioning voltages are 2.9V */
	/*  and 3.0V respectively. */
	/*  */
	/*  Note that the hardware has a minimum resolution of 8mV. If */
	/*  the given voltage cannot be expressed exactly, then the */
	/*  smallest expressible larger value will be used. */

	uint16_t u16ConditioningMaxVoltage;

	/* brief Units in milliamps. */
	/*  */
	/*  This field configures the maximum conditioning current. */
	/*  This is the maximum current that will be offered to a */
	/*  battery while it is being conditioned. A typical value is */
	/*  0.1C. */
	/*  */
	/*  Note that the hardware has a minimum resolution of 10mA */
	/*  (see the data sheet for details). If the given current */
	/*  cannot be expressed exactly, then the largest expressible */
	/*  smaller value will be used. */

	uint16_t u16ConditioningCurrent;

	/* brief Units in milliseconds. */
	/*  */
	/*  This field configures the conditioning time-out. This is */
	/*  the maximum amount of time that a battery will be */
	/*  conditioned before the battery charger declares it to be */
	/*  broken. */
	/*  */
	/*  Note that the minimum time resolution of the battery */
	/*  charger is the configured period between calls to advance */
	/*  the state machine. If the given time-out cannot be */
	/*  expressed exactly, then the shortest expressible longer */
	/*  value will be used. */

	uint32_t u32ConditioningTimeout;

	/* brief Units in millivolts. */
	/*  */
	/*  This field configures the final charging voltage. At this */
	/*  writing, only two values are permitted: 4100 or 4200. */

	uint16_t u16ChargingVoltage;

	/* brief Units in milliamps. */
	/*  */
	/*  This field configures the maximum current offered to a */
	/*  charging battery. */
	/*  */
	/*  Note that the hardware has a minimum resolution of 10mA */
	/*  (see the data sheet for details). If the given current */
	/*  cannot be expressed exactly, then the largest expressible */
	/*  smaller value will be used. */

	uint16_t u16ChargingCurrent;

	/* brief Units in milliamps. */
	/*  */
	/*  This field configures the current flow below which a */
	/*  charging battery is regarded as fully charged (typical */
	/*  0.1C). At this point, the battery will be topped off. */
	/*  */
	/*  Note that the hardware has a minimum resolution of 10mA */
	/*  (see the data sheet for details). If the given current */
	/*  cannot be expressed exactly, then the largest expressible */
	/*  smaller value will be used. */

	uint16_t u16ChargingThresholdCurrent;

	/* brief Units in milliamps. */
	/*  */
	/*  When charging while the DCDC converter's are enabled, the charger */
	/*  is suppling current to both the battery and the Vbat input of the */
	/*  DCDC converter.  Once the total battery charger current falls */
	/*  below this level, the charger will then stop charging until the */
	/*  the battery voltage reaches the BC_LOW_DCDCMODE_BATTERY_VOLTAGE */
	/*  threshold or until the DCDCs are no longer enabled. */
	/*  */
	/*  Typically, this value should be left at 180 to avoid the risk */
	/*  of topping off the battery too long in DCDC mode and avoid */
	/*  exceeding the BC_CHARGING_TIMEOUT time which would put the charger */
	/*  driver in the broken state and completely disable charging. */
	/*  */
	/*  Note that the hardware has a minimum resolution of 10mA */
	/*  (see the data sheet for details). If the given current */
	/*  cannot be expressed exactly, then the largest expressible */
	/*  smaller value will be used. */
	uint16_t u16DdcdModeChargingThresholdCurrent;

	/* brief Units in milliseconds. */
	/*  */
	/*  This field configures the charging time-out. This is the */
	/*  maximum amount of time that a battery will be charged */
	/*  before the battery charger declares it to be broken. */
	/*  */
	/*  Note that the minimum time resolution of the battery */
	/*  charger is the configured period between calls to advance */
	/*  the state machine. If the given time-out cannot be */
	/*  expressed exactly, then the shortest expressible longer */
	/*  value will be used. */

	uint32_t u32ChargingTimeout;

	/* brief Units in milliseconds. */
	/*  */
	/*  This field configures the top-off period. This is the */
	/*  amount of time a battery will be held in the Topping Off */
	/*  state before it is declared fully charged. */
	/*  */
	/*  Note that the minimum time resolution of the battery */
	/*  charger is the configured period between calls to advance */
	/*  the state machine. If the given time-out cannot be */
	/*  expressed exactly, then the shortest expressible longer */
	/*  value will be used. */

	uint32_t u32TopOffPeriod;

	/* brief Units in milliseconds. */
	/*  */
	/*  This field configures the top-off period when the DCDC */
	/*  converters are enabled. To avoid topping off the LiIon */
	/*  battery too long and reducing it's long term capacity, */
	/*  This time should be kept failry short. */
	/*  */
	/*  Note that the minimum time resolution of the battery */
	/*  charger is the configured period between calls to advance */
	/*  the state machine. If the given time-out cannot be */
	/*  expressed exactly, then the shortest expressible longer */
	/*  value will be used. */
	uint32_t u32DcdcModeTopOffPeriod;

	/* brief Causes the battery charger to use an externally generated bias current */
	/*  */
	/*  If cleared, this causes the battery charger to use an */
	/*  externally generated bias current, which is expected to be */
	/*  quite precise. Otherwise, the battery charger will */
	/*  generate a lesser-quality bias current internally. */

	uint8_t useInternalBias:1;

	/* brief Indicates that the battery charger is to monitor the die temperature. */
	/*  */
	/*  If set, this field indicates that the battery charger is to */
	/*  monitor the die temperature. See below for fields that */
	/*  configure the details. */

	uint8_t monitorDieTemp:1;

	/* brief Indicates that the battery charger is to monitor the battery temperature. */
	/*  */
	/*  If set, this field indicates that the battery charger is to */
	/*  monitor the battery temperature. See below for fields that */
	/*  configure the details. */

	uint8_t monitorBatteryTemp:1;

	/* brief Units in degrees centigrade. */
	/*  */
	/*  Note that the hardware reports die temperature in ranges of */
	/*  10 degree resolution minimum (see the data sheet for */
	/*  details). If the battery charger is monitoring the die */
	/*  temperature, and it rises to a range that includes a */
	/*  temperature greater than or equal to this value, the */
	/*  charging current will be clamped to the safe current. */

	int8_t u8DieTempHigh;

	/* brief Units in degrees centigrade. */
	/*  */
	/*  Note that the hardware reports die temperature in ranges of */
	/*  10 degrees minimum (see the data sheet for details). If the */
	/*  charging current is being clamped because of a high die */
	/*  temperature, and it falls to a range that doesn't include a */
	/*  temperatures greater than or equal to this value, the */
	/*  charging current clamp will be released. */

	int8_t u8DieTempLow;

	/* brief Units in milliamps. */
	/*  */
	/*  If the battery charger detects a high die temperature, it */
	/*  will clamp the charging current at or below this value. */

	uint16_t u16DieTempSafeCurrent;

	/* brief This field indicates the LRADC channel to
     read Die temperature */

	uint8_t u8DieTempChannel;


	/* brief If the battery charger is monitoring the battery */
	/*  temperature, this field indicates the LRADC channel to */
	/*  read. */

	uint8_t u8BatteryTempChannel;

	/* brief If the battery charger is monitoring the battery */
	/*  temperature, and it rises to a measurement greater than or */
	/*  equal to this value, the charging current will be clamped */
	/*  to the corresponding safe current. */

	uint16_t u16BatteryTempHigh;

	/* brief If the charging current is being clamped because of a high */
	/*  battery temperature, and it falls below this value, the */
	/*  charging current clamp will be released. */

	uint16_t u16BatteryTempLow;

	/* brief Units in milliamps. */
	/*  */
	/*  If the battery charger detects a high battery temperature, */
	/*  it will clamp the charging current at or below this value. */

	uint16_t u16BatteryTempSafeCurrent;

	/* brief Units in millivolts. */
	/*  */
	/*  In the WaitingToCharge state, if we are in DCDC */
	/*  operating modes, if the battery voltage measurement */
	/*  is below this value, we immediately proceed with charging. */
	/*  the low criteria for this value is that it must be high */
	/*  to not risk the battery voltage getting too low.  The */
	/*  upper criteria is that you do not want the IR voltage */
	/*  drop under heavy loads to make you start charging too soon */
	/*  because the goal in DCDC operating mode is to not be constantly */
	/*  topping off the battery which can shorten its life */

	uint16_t u16LowDcdcBatteryVoltage_mv;

	uint32_t u32StateMachineNonChargingPeriod;
} ddi_bc_Cfg_t;

/*  Status returned by Battery Charger functions. */

typedef enum _ddi_bc_Status {
	/* brief TBD */
	DDI_BC_STATUS_SUCCESS = 0,
	/* brief TBD */
	DDI_BC_STATUS_HARDWARE_DISABLED,
	/* brief TBD */
	DDI_BC_STATUS_BAD_BATTERY_MODE,
	/* brief TBD */
	DDI_BC_STATUS_CLOCK_GATE_CLOSED,
	/* brief TBD */
	DDI_BC_STATUS_NOT_INITIALIZED,
	/* brief TBD */
	DDI_BC_STATUS_ALREADY_INITIALIZED,
	/* brief TBD */
	DDI_BC_STATUS_BROKEN,
	/* brief TBD */
	DDI_BC_STATUS_NOT_BROKEN,
	/* brief TBD */
	DDI_BC_STATUS_NOT_DISABLED,
	/* brief TBD */
	DDI_BC_STATUS_BAD_ARGUMENT,
	/* brief TBD */
	DDI_BC_STATUS_CFG_BAD_BATTERY_TEMP_CHANNEL,
	/* brief TBD */
	DDI_BC_STATUS_CFG_BAD_CHARGING_VOLTAGE,
} ddi_bc_Status_t;


/*  BCM Event Codes */

/* These are the codes that might be published to PMI Subscribers. */


#define DDI_BC_EVENT_GROUP (11<<10)

/* brief TBD */
/* todo [PUBS] Add definition(s)... */
typedef enum {
	/* Use the error code group value to make events unique for the EOI */
	/* brief TBD */
	ddi_bc_MinEventCode = DDI_BC_EVENT_GROUP,
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


/* Prototypes */



/* brief Initialize the Battery Charger. */
/*  */
/* fntype Function */
/*  */
/*  This function initializes the Battery Charger. */
/*  */
/* param[in]  pCfg  A pointer to the new configuration. */
/*  */
/* retval  DDI_BC_STATUS_SUCCESS */
/*              If the operation succeeded. */
/* retval  DDI_BC_STATUS_ALREADY_INITIALIZED */
/*              If the Battery Charger is already initialized. */
/* retval  DDI_BC_STATUS_HARDWARE_DISABLED */
/*              If the Battery Charger hardware is disabled by a laser fuse. */
/* retval  DDI_BC_STATUS_BAD_BATTERY_MODE */
/*              If the power supply is set up for a non-rechargeable battery. */
/* retval  DDI_BC_STATUS_CLOCK_GATE_CLOSED */
/*              If the clock gate for the power supply registers is closed. */
/* retval  DDI_BC_STATUS_CFG_BAD_CHARGING_VOLTAGE */
/*              If the charging voltage is not either 4100 or 4200. */
/* retval  DDI_BC_STATUS_CFG_BAD_BATTERY_TEMP_CHANNEL */
/*              If the LRADC channel number for monitoring battery temperature */
/*              is bad. */
/*  */
/* internal */
/* see To view the function definition, see ddi_bc_init.c. */

extern ddi_bc_Status_t ddi_bc_Init(ddi_bc_Cfg_t *pCfg);

/*  */
/* brief Report the Battery Charger configuration. */
/*  */
/* fntype Function */
/*  */
/*  This function reports the Battery Charger configuration. */
/*  */
/*  Note that, if the Battery Charger has not yet been initialized, the data */
/*  returned by this function is unknown. */
/*  */
/* param[in,out]  pCfg  A pointer to a structure that will receive the data. */
/*  */
/* internal */
/* see To view the function definition, see ddi_bc_api.c. */

extern void ddi_bc_QueryCfg(ddi_bc_Cfg_t *pCfg);

/*  */
/* brief Shut down the Battery Charger. */
/*  */
/* fntype Function */
/*  */
/*  This function immediately shuts down the Battery Charger hardware and */
/*  returns the state machine to the Uninitialized state. Use this function to */
/*  safely mummify the battery charger before retiring it from memory. */
/*  */
/* internal */
/* see To view the function definition, see ddi_bc_api.c. */

extern void ddi_bc_ShutDown(void);

/*  */
/* brief Advances the state machine. */
/*  */
/* fntype Function */
/*  */
/*  This function advances the state machine. */
/*  */
/* retval DDI_BC_STATUS_SUCCESS          If all goes well */
/* retval DDI_BC_STATUS_NOT_INITIALIZED  If the Battery Charger is not yet */
/*                                         initialized. */
/* retval DDI_BC_STATUS_BROKEN           If the battery violated a time-out */
/*                                         and has been declared broken. */
/*  */
/* internal */
/* see To view the function definition, see ddi_bc_api.c. */

extern ddi_bc_Status_t ddi_bc_StateMachine(void);

/*  */
/* brief Get the Battery Charger's current state. */
/*  */
/* fntype Function */
/*  */
/*  This function returns the current state. */
/*  */
/* retval The current state. */
/*  */
/* internal */
/* see To view the function definition, see ddi_bc_api.c. */

extern ddi_bc_State_t ddi_bc_GetState(void);

/*  */
/* brief Disable the Battery Charger. */
/*  */
/* fntype Function */
/*  */
/*  This function forces the Battery Charger into the Disabled state. */
/*  */
/* retval DDI_BC_STATUS_SUCCESS          If all goes well */
/* retval DDI_BC_STATUS_NOT_INITIALIZED  If the Battery Charger is not yet */
/*                                         initialized. */
/*  */
/* internal */
/* see To view the function definition, see ddi_bc_api.c. */

extern ddi_bc_Status_t ddi_bc_SetDisable(void);

/*  */
/* brief Enable the Battery Charger. */
/*  */
/* fntype Function */
/*  */
/*  If the Battery Charger is in the Disabled state, this function moves it to */
/*  the Waiting to Charge state. */
/*  */
/* retval DDI_BC_STATUS_SUCCESS          If all goes well */
/* retval DDI_BC_STATUS_NOT_INITIALIZED  If the Battery Charger is not yet */
/*                                         initialized. */
/* retval DDI_BC_STATUS_NOT_DISABLED     If the Battery Charger is not */
/*                                         disabled. */
/*  */
/* internal */
/* see To view the function definition, see ddi_bc_api.c. */

extern ddi_bc_Status_t ddi_bc_SetEnable(void);

/*  */
/* brief Declare the battery to be broken. */
/*  */
/* fntype Function */
/*  */
/*  This function forces the Battery Charger into the Broken state. */
/*  */
/* retval DDI_BC_STATUS_SUCCESS          If all goes well */
/* retval DDI_BC_STATUS_NOT_INITIALIZED  If the Battery Charger is not yet */
/*                                         initialized. */
/*  */
/* internal */
/* see To view the function definition, see ddi_bc_api.c. */

extern ddi_bc_Status_t ddi_bc_SetBroken(void);

/*  */
/* brief Declare the battery to be fixed. */
/*  */
/* fntype Function */
/*  */
/*  If the Battery Charger is in the Broken state, this function moves it to */
/*  the Disabled state. */
/*  */
/* retval DDI_BC_STATUS_SUCCESS          If all goes well */
/* retval DDI_BC_STATUS_NOT_INITIALIZED  If the Battery Charger is not yet */
/*                                         initialized. */
/* retval DDI_BC_STATUS_NOT_BROKEN       If the Battery Charger is not broken. */
/*  */
/* internal */
/* see To view the function definition, see ddi_bc_api.c. */

extern ddi_bc_Status_t ddi_bc_SetFixed(void);

/*  */
/* brief Set the current limit. */
/*  */
/* fntype Function */
/*  */
/*  This function applies a limit to the current that the Battery Charger can */
/*  draw. */
/*  */
/* param[in]  u16Limit  The maximum current the Battery Charger can draw */
/*                        (in mA). */
/*  */
/* retval  The expressible version of the limit. */
/*  */
/* internal */
/* see To view the function definition, see ddi_bc_api.c. */

extern uint16_t ddi_bc_SetCurrentLimit(uint16_t u16Limit);


/*  */
/* brief Report the current limit. */
/*  */
/* fntype Function */
/*  */
/*  This function reports the limit to the current that the Battery Charger can */
/*  draw. */
/*  */
/* retval  The current limit. */
/*  */
/* internal */
/* see To view the function definition, see ddi_bc_api.c. */

extern uint16_t ddi_bc_GetCurrentLimit(void);


/*  */
/* brief Set the current threshold. */
/*  */
/* fntype Function */
/*  */
/*  */
/* param[in]  u16Current Current threshold where charger deactivates (in mA). */
/*  */
/*  */

extern uint16_t ddi_bc_SetCurrentThreshold(uint16_t u16Current);


/*  */
/* brief Set the battery charger state machine period. */
/*  */
/* fntype Function */
/*  */
/*  This function sets a new state machine period.  The Period and Slope should */
/*  be coordinated to achieve the minimal ramp step current which will minimize */
/*  transients on the system. */
/*  */
/* param[in]  u32StateMachinePeriod  (in milliseconds) */
/* param[in]  u16CurrentRampSlope (in mA/s) */
/*  */
/* retval SUCCESS                        If all goes well */
/* retval ERROR_DDI_BCM_NOT_INITIALIZED  If the Battery Charger is not yet */
/*                                         initialized. */
/*  */

extern ddi_bc_Status_t ddi_bc_SetNewPeriodAndSlope(uint32_t
						   u32StateMachinePeriod,
						   uint16_t
						   u16CurrentRampSlope);


/*  */
/* brief Report the state machine period. */
/*  */
/* fntype Function */
/*  */
/*  This function reports the battery charger period. */
/*  */
/* retval  The battery charger period (in milliseconds). */
/*  */

extern uint32_t ddi_bc_GetStateMachinePeriod(void);


/*  */
/* brief Report the current ramp slope. */
/*  */
/* fntype Function */
/*  */
/*  This function reports the current ramp slope. */
/*  */
/* retval  The current ramp slope (in mA/s). */
/*  */

extern uint32_t ddi_bc_GetCurrentRampSlope(void);


/*  */
/* brief Report the time spent in the present state (milliseconds) */
/*  */
/* fntype Function */
/*  */
/*  This function reports the time spent in the present charging state.  Note that */
/*  for the states that actually charge the battery, this time does not include the */
/*  time spent under alarm conditions such as die termperature alarm or battery */
/*  temperature alarm. */
/*  */
/* retval  The time spent in the current state in milliseconds. */
/*  */

uint32_t ddi_bc_GetStateTime(void);


/*  */
/* brief Report the reason for being in the broken state */
/*  */
/* fntype Function */
/*  */
/*  */
/* retval  ddi_bc_BrokenReason_t enumeration */
/*  */

ddi_bc_BrokenReason_t ddi_bc_GetBrokenReason(void);


/*  */
/* brief Restart the charge cycle */
/*  */
/* fntype Function */
/*  */
/* retval  SUCCESS */
/*  */

ddi_bc_Status_t ddi_bc_ForceChargingToStart(void);

void fsl_enable_usb_plugindetect(void);

int fsl_is_usb_plugged(void);

/* End of file */

#endif				/* _DDI_BC_H */
/*  @} */
