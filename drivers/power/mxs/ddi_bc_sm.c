/*
 * Copyright (C) 2010 Freescale Semiconductor, Inc.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */


/* addtogroup ddi_bc */
/*  @{ */
/*  */
/* Copyright (c) 2004-2005 SigmaTel, Inc. */
/*  */
/* file       ddi_bc_sm.c */
/* brief      Contains the Battery Charger state machine. */



/* Includes */


#include <mach/ddi_bc.h>
#include "ddi_bc_internal.h"

#include <linux/delay.h>


/* Definitions */


/* This is the minimum time we must charge before we transition from */
/* the charging state to the topping off. If we reach the */
/* u16ChargingThresholdCurrent charge curent before then, the battery was */
/* already full so we can avoid the risk of charging it past .1C for */
/* too long. */

#define TRANSITION_TO_TOPOFF_MINIMUM_CHARGE_TIME_mS (1 * 60 * 1000)	/* 1 minute */


/* Variables */


/* The current state. */

ddi_bc_State_t g_ddi_bc_State = DDI_BC_STATE_UNINITIALIZED;

/* This table contains pointers to the functions that implement states. The */
/* table is indexed by state. Note that it's critically important for this */
/* table to agree with the state enumeration in ddi_bc.h. */

static ddi_bc_Status_t ddi_bc_Uninitialized(void);
static ddi_bc_Status_t ddi_bc_Broken(void);
static ddi_bc_Status_t ddi_bc_Disabled(void);
static ddi_bc_Status_t ddi_bc_WaitingToCharge(void);
static ddi_bc_Status_t ddi_bc_Conditioning(void);
static ddi_bc_Status_t ddi_bc_Charging(void);
static ddi_bc_Status_t ddi_bc_ToppingOff(void);


ddi_bc_Status_t(*const (stateFunctionTable[])) (void) = {
ddi_bc_Uninitialized,
	    ddi_bc_Broken,
	    ddi_bc_Disabled,
	    ddi_bc_WaitingToCharge,
	    ddi_bc_Conditioning,
	    ddi_bc_Charging, ddi_bc_ToppingOff};

/* Used by states that need to watch the time. */
uint32_t g_ddi_bc_u32StateTimer;

/* Always attempt to charge on first 5V connection */
bool bRestartChargeCycle = true;

#ifdef CONFIG_POWER_SUPPLY_DEBUG
static uint16_t u16ExternalBatteryPowerVoltageCheck;
#endif

ddi_bc_BrokenReason_t ddi_bc_gBrokenReason = DDI_BC_BROKEN_UNINITIALIZED;


/* Code */



/*  */
/* brief Transition to the Waiting to Charge state. */
/*  */
/* fntype Function */
/*  */
/*  This function implements the transition to the Waiting to Charge state. */
/*  */

static void TransitionToWaitingToCharge(void)
{

	/* -------------------------------------------------------------------------- */
	/* Reset the state timer. */
	/* -------------------------------------------------------------------------- */

	g_ddi_bc_u32StateTimer = 0;

	/* -------------------------------------------------------------------------- */
	/* Reset the current ramp. */
	/* -------------------------------------------------------------------------- */

	ddi_bc_RampReset();

	/* -------------------------------------------------------------------------- */
	/* Move to the Waiting to Charge state. */
	/* -------------------------------------------------------------------------- */

	g_ddi_bc_State = DDI_BC_STATE_WAITING_TO_CHARGE;

#ifdef CONFIG_POWER_SUPPLY_DEBUG
	printk("Battery charger: now waiting to charge\n");
#endif

}


/*  */
/* brief Transition to the Conditioning state. */
/*  */
/* fntype Function */
/*  */
/*  This function implements the transition to the Conditioning state. */
/*  */

static void TransitionToConditioning(void)
{

	/* -------------------------------------------------------------------------- */
	/* Reset the state timer. */
	/* -------------------------------------------------------------------------- */

	g_ddi_bc_u32StateTimer = 0;

	/* -------------------------------------------------------------------------- */
	/* Set up the current ramp for conditioning. */
	/* -------------------------------------------------------------------------- */

	ddi_bc_RampSetTarget(g_ddi_bc_Configuration.u16ConditioningCurrent);

	/* -------------------------------------------------------------------------- */
	/* Move to the Conditioning state. */
	/* -------------------------------------------------------------------------- */

	g_ddi_bc_State = DDI_BC_STATE_CONDITIONING;

#ifdef CONFIG_POWER_SUPPLY_DEBUG
	printk("Battery charger: now conditioning\n");
#endif

}


/*  */
/* brief Transition to the Charging state. */
/*  */
/* fntype Function */
/*  */
/*  This function implements the transition to the Charging state. */
/*  */

static void TransitionToCharging(void)
{

	/* -------------------------------------------------------------------------- */
	/* Reset the state timer. */
	/* -------------------------------------------------------------------------- */

	g_ddi_bc_u32StateTimer = 0;

	/* -------------------------------------------------------------------------- */
	/* Set up the current ramp for charging. */
	/* -------------------------------------------------------------------------- */

	ddi_bc_RampSetTarget(g_ddi_bc_Configuration.u16ChargingCurrent);

	/* -------------------------------------------------------------------------- */
	/* We'll be finished charging when the current flow drops below this level. */
	/* -------------------------------------------------------------------------- */

	ddi_bc_hwSetCurrentThreshold(g_ddi_bc_Configuration.
				     u16ChargingThresholdCurrent);

	/* -------------------------------------------------------------------------- */
	/* Move to the Charging state. */
	/* -------------------------------------------------------------------------- */

	g_ddi_bc_State = DDI_BC_STATE_CHARGING;
#ifdef CONFIG_POWER_SUPPLY_DEBUG
	printk("Battery charger: now charging\n");
#endif
}


/*  */
/* brief Transition to the Topping Off state. */
/*  */
/* fntype Function */
/*  */
/*  This function implements the transition to the Topping Off state. */
/*  */

static void TransitionToToppingOff(void)
{

	/* -------------------------------------------------------------------------- */
	/* Reset the state timer. */
	/* -------------------------------------------------------------------------- */

	g_ddi_bc_u32StateTimer = 0;

	/* -------------------------------------------------------------------------- */
	/* Set up the current ramp for topping off. */
	/* -------------------------------------------------------------------------- */

	ddi_bc_RampSetTarget(g_ddi_bc_Configuration.u16ChargingCurrent);

	/* -------------------------------------------------------------------------- */
	/* Move to the Topping Off state. */
	/* -------------------------------------------------------------------------- */

	g_ddi_bc_State = DDI_BC_STATE_TOPPING_OFF;

#ifdef CONFIG_POWER_SUPPLY_DEBUG
	printk("Battery charger: now topping off\n");
#endif

}


/*  */
/* brief Transition to the Broken state. */
/*  */
/* fntype Function */
/*  */
/*  This function implements the transition to the Broken state. */
/*  */

static void TransitionToBroken(void)
{

	/* -------------------------------------------------------------------------- */
	/* Reset the state timer. */
	/* -------------------------------------------------------------------------- */

	g_ddi_bc_u32StateTimer = 0;

	/* -------------------------------------------------------------------------- */
	/* Reset the current ramp. */
	/* -------------------------------------------------------------------------- */

	ddi_bc_RampReset();

	/* -------------------------------------------------------------------------- */
	/* Move to the Broken state. */
	/* -------------------------------------------------------------------------- */

	g_ddi_bc_State = DDI_BC_STATE_BROKEN;

	pr_info("charger------ ddi_bc_gBrokenReason=%d\n",
		ddi_bc_gBrokenReason);
#ifdef CONFIG_POWER_SUPPLY_DEBUG
	printk("Battery charger: declaring a broken battery\n");
#endif

}


/*  */
/* brief Uninitialized state function. */
/*  */
/* fntype Function */
/*  */
/*  This function implements the Uninitialized state. */
/*  */

static ddi_bc_Status_t ddi_bc_Uninitialized(void)
{

	/* -------------------------------------------------------------------------- */
	/* The first order of business is to update alarms. */
	/* -------------------------------------------------------------------------- */

	ddi_bc_RampUpdateAlarms();

	/* -------------------------------------------------------------------------- */
	/* Increment the state timer. */
	/* -------------------------------------------------------------------------- */

	g_ddi_bc_u32StateTimer += g_ddi_bc_Configuration.u32StateMachinePeriod;

	/* -------------------------------------------------------------------------- */
	/* The only way to leave this state is with a call to ddi_bc_Initialize. So, */
	/* calling this state function does nothing. */
	/* -------------------------------------------------------------------------- */

	return DDI_BC_STATUS_SUCCESS;

}


/*  */
/* brief Broken state function. */
/*  */
/* fntype Function */
/*  */
/*  This function implements the Broken state. */
/*  */

static ddi_bc_Status_t ddi_bc_Broken(void)
{

	/* -------------------------------------------------------------------------- */
	/* The first order of business is to update alarms. */
	/* -------------------------------------------------------------------------- */

	ddi_bc_RampUpdateAlarms();

	/* -------------------------------------------------------------------------- */
	/* Increment the state timer. */
	/* -------------------------------------------------------------------------- */

	g_ddi_bc_u32StateTimer += g_ddi_bc_Configuration.u32StateMachinePeriod;

	/* -------------------------------------------------------------------------- */
	/* The only way to leave this state is with a call to ddi_bc_SetFixed. So, */
	/* calling this state function does nothing. */
	/* -------------------------------------------------------------------------- */

	return DDI_BC_STATUS_SUCCESS;

}


/*  */
/* brief Disabled state function. */
/*  */
/* fntype Function */
/*  */
/*  This function implements the Disabled state. */
/*  */

static ddi_bc_Status_t ddi_bc_Disabled(void)
{

	/* -------------------------------------------------------------------------- */
	/* The first order of business is to update alarms. */
	/* -------------------------------------------------------------------------- */

	ddi_bc_RampUpdateAlarms();

	/* -------------------------------------------------------------------------- */
	/* Increment the state timer. */
	/* -------------------------------------------------------------------------- */

	g_ddi_bc_u32StateTimer += g_ddi_bc_Configuration.u32StateMachinePeriod;

	/* -------------------------------------------------------------------------- */
	/* The only way to leave this state is with a call to ddi_bc_SetEnable. So, */
	/* calling this state function does nothing. */
	/* -------------------------------------------------------------------------- */

	return DDI_BC_STATUS_SUCCESS;

}


/*  */
/* brief Waitin to Charge state function. */
/*  */
/* fntype Function */
/*  */
/*  This function implements the Waiting to Charge state. */
/*  */

static ddi_bc_Status_t ddi_bc_WaitingToCharge(void)
{
	uint16_t u16BatteryVoltage;
	/* -------------------------------------------------------------------------- */
	/* The first order of business is to update alarms. */
	/* -------------------------------------------------------------------------- */

	ddi_bc_RampUpdateAlarms();

	/* -------------------------------------------------------------------------- */
	/* Increment the state timer. */
	/* -------------------------------------------------------------------------- */

	g_ddi_bc_u32StateTimer += g_ddi_bc_Configuration.u32StateMachinePeriod;

	/* -------------------------------------------------------------------------- */
	/* Check if the power supply is present. If not, we're not going anywhere. */
	/* -------------------------------------------------------------------------- */

	if (!ddi_bc_hwPowerSupplyIsPresent()) {
#ifdef CONFIG_POWER_SUPPLY_DEBUG
		u16ExternalBatteryPowerVoltageCheck = 0;
#endif
		return DDI_BC_STATUS_SUCCESS;
	}
	/* -------------------------------------------------------------------------- */
	/* If control arrives here, we're connected to a power supply. Have a look */
	/* at the battery voltage. */
	/* -------------------------------------------------------------------------- */

	u16BatteryVoltage = ddi_bc_hwGetBatteryVoltage();

#ifdef CONFIG_POWER_SUPPLY_DEBUG
	if (u16ExternalBatteryPowerVoltageCheck) {
		if ((u16ExternalBatteryPowerVoltageCheck - u16BatteryVoltage) >
		    300) {
			/*
			 * If control arrives here, battery voltage has
			 * dropped too quickly after the first charge
			 * cycle.  We think an external voltage regulator is
			 * connected.
			 */

			ddi_bc_gBrokenReason =
			    DDI_BC_BROKEN_EXTERNAL_BATTERY_VOLTAGE_DETECTED;

			TransitionToBroken();

			/* ---------------------------------------------------------------------- */
			/* Tell our caller the battery appears to be broken. */
			/* ---------------------------------------------------------------------- */

			return DDI_BC_STATUS_BROKEN;
		} else {
			/* reset this check */
			u16ExternalBatteryPowerVoltageCheck = 0;
		}

	}
#endif


	/* -------------------------------------------------------------------------- */
	/* If the battery voltage isn't low, we don't need to be charging it. We */
	/* use a 5% margin to decide. */
	/* -------------------------------------------------------------------------- */

	if (!bRestartChargeCycle) {
		uint16_t x;

		x = u16BatteryVoltage + (u16BatteryVoltage / 20);

		if (x >= g_ddi_bc_Configuration.u16ChargingVoltage)
			return DDI_BC_STATUS_SUCCESS;

	}

	bRestartChargeCycle = false;
	/* -------------------------------------------------------------------------- */
	/* If control arrives here, the battery is low. How low? */
	/* -------------------------------------------------------------------------- */

	if (u16BatteryVoltage <
	    g_ddi_bc_Configuration.u16ConditioningThresholdVoltage) {

		/* ---------------------------------------------------------------------- */
		/* If control arrives here, the battery is very low and it needs to be */
		/* conditioned. */
		/* ---------------------------------------------------------------------- */

		TransitionToConditioning();

	} else {

		/* ---------------------------------------------------------------------- */
		/* If control arrives here, the battery isn't too terribly low. */
		/* ---------------------------------------------------------------------- */

		TransitionToCharging();

	}

	/* -------------------------------------------------------------------------- */
	/* Return success. */
	/* -------------------------------------------------------------------------- */

	return DDI_BC_STATUS_SUCCESS;

}


/*  */
/* brief Conditioning state function. */
/*  */
/* fntype Function */
/*  */
/*  This function implements the Conditioning state. */
/*  */

static ddi_bc_Status_t ddi_bc_Conditioning(void)
{

	/* -------------------------------------------------------------------------- */
	/* The first order of business is to update alarms. */
	/* -------------------------------------------------------------------------- */

	ddi_bc_RampUpdateAlarms();

	/* -------------------------------------------------------------------------- */
	/* If we're not under an alarm, increment the state timer. */
	/* -------------------------------------------------------------------------- */

	if (!ddi_bc_RampGetDieTempAlarm() && !ddi_bc_RampGetBatteryTempAlarm()) {
		g_ddi_bc_u32StateTimer +=
		    g_ddi_bc_Configuration.u32StateMachinePeriod;
	}
	/* -------------------------------------------------------------------------- */
	/* Check if the power supply is still around. */
	/* -------------------------------------------------------------------------- */

	if (!ddi_bc_hwPowerSupplyIsPresent()) {

		/* ---------------------------------------------------------------------- */
		/* If control arrives here, the power supply has been removed. Go back */
		/* and wait. */
		/* ---------------------------------------------------------------------- */

		TransitionToWaitingToCharge();

		/* ---------------------------------------------------------------------- */
		/* Return success. */
		/* ---------------------------------------------------------------------- */

		return DDI_BC_STATUS_SUCCESS;

	}

	/* -------------------------------------------------------------------------- */
	/* If control arrives here, we're still connected to a power supply. */
	/* Check if a battery is connected.  If the voltage rises to high with only */
	/* conditioning charge current, we determine that a battery is not connected. */
	/* If that is not the case and a battery is connected, check */
	/* if the battery voltage indicates it still needs conditioning. */
	/* -------------------------------------------------------------------------- */

/* 	if (ddi_bc_hwGetBatteryVoltage() >= 3900) { */
	if ((ddi_bc_hwGetBatteryVoltage() >
	     g_ddi_bc_Configuration.u16ConditioningMaxVoltage) &&
	    (ddi_power_GetMaxBatteryChargeCurrent() <
	     g_ddi_bc_Configuration.u16ConditioningCurrent)) {
		/* ---------------------------------------------------------------------- */
		/* If control arrives here, voltage has risen too quickly for so */
		/* little charge being applied so their must be no battery connected. */
		/* ---------------------------------------------------------------------- */

		ddi_bc_gBrokenReason = DDI_BC_BROKEN_NO_BATTERY_DETECTED;

		TransitionToBroken();

		/* ---------------------------------------------------------------------- */
		/* Tell our caller the battery appears to be broken. */
		/* ---------------------------------------------------------------------- */

		return DDI_BC_STATUS_BROKEN;

	}

	if (ddi_bc_hwGetBatteryVoltage() >=
	    g_ddi_bc_Configuration.u16ConditioningMaxVoltage) {

		/* ---------------------------------------------------------------------- */
		/* If control arrives here, this battery no longer needs conditioning. */
		/* ---------------------------------------------------------------------- */

		TransitionToCharging();

		/* ---------------------------------------------------------------------- */
		/* Return success. */
		/* ---------------------------------------------------------------------- */

		return DDI_BC_STATUS_SUCCESS;

	}
	/* -------------------------------------------------------------------------- */
	/* Have we been in this state too long? */
	/* -------------------------------------------------------------------------- */

	if (g_ddi_bc_u32StateTimer >=
	    g_ddi_bc_Configuration.u32ConditioningTimeout) {

		/* ---------------------------------------------------------------------- */
		/* If control arrives here, we've been here too long. */
		/* ---------------------------------------------------------------------- */

		ddi_bc_gBrokenReason = DDI_BC_BROKEN_CHARGING_TIMEOUT;

		TransitionToBroken();

		/* ---------------------------------------------------------------------- */
		/* Tell our caller the battery appears to be broken. */
		/* ---------------------------------------------------------------------- */

		return DDI_BC_STATUS_BROKEN;

	}
	/* -------------------------------------------------------------------------- */
	/* If control arrives here, we're staying in this state. Step the current */
	/* ramp. */
	/* -------------------------------------------------------------------------- */

	ddi_bc_RampStep(g_ddi_bc_Configuration.u32StateMachinePeriod);

	/* -------------------------------------------------------------------------- */
	/* Return success. */
	/* -------------------------------------------------------------------------- */

	return DDI_BC_STATUS_SUCCESS;

}


/*  */
/* brief Charging state function. */
/*  */
/* fntype Function */
/*  */
/*  This function implements the Charging state. */
/*  */

static ddi_bc_Status_t ddi_bc_Charging(void)
{

	/* -------------------------------------------------------------------------- */
	/* This variable counts the number of times we've seen the charging status */
	/* bit cleared. */
	/* -------------------------------------------------------------------------- */

	static int iStatusCount;
	/* -------------------------------------------------------------------------- */
	/* The first order of business is to update alarms. */
	/* -------------------------------------------------------------------------- */

	ddi_bc_RampUpdateAlarms();

	/* -------------------------------------------------------------------------- */
	/* If we're not under an alarm, increment the state timer. */
	/* -------------------------------------------------------------------------- */

	if (!ddi_bc_RampGetDieTempAlarm() && !ddi_bc_RampGetBatteryTempAlarm()) {
		g_ddi_bc_u32StateTimer +=
		    g_ddi_bc_Configuration.u32StateMachinePeriod;
	}
	/* Check if the power supply is still around. */


	if (!ddi_bc_hwPowerSupplyIsPresent()) {

		/* ---------------------------------------------------------------------- */
		/* If control arrives here, the power supply has been removed. Go back */
		/* and wait. */
		/* ---------------------------------------------------------------------- */

		TransitionToWaitingToCharge();

		/* ---------------------------------------------------------------------- */
		/* Return success. */
		/* ---------------------------------------------------------------------- */

		return DDI_BC_STATUS_SUCCESS;

	}
	/* -------------------------------------------------------------------------- */
	/* If control arrives here, we're still connected to a power supply. We need */
	/* to decide now if the battery is still charging, or if it's nearly full. */
	/* If it's still charging, we'll stay in this state. Otherwise, we'll move */
	/* to the Topping Off state. */
	/*  */
	/* Most of the time, we decide that the battery is still charging simply by */
	/* checking if the the actual current flow is above the charging threshold */
	/* current (as indicated by the charge status bit). However, if we're */
	/* still ramping up to full charging current, the hardware may still be set */
	/* to deliver an amount that's less than the threshold. In that case, the */
	/* charging status bit would *definitely* show a low charging current, but */
	/* that doesn't mean the battery is ready for topping off. */
	/*  */
	/* So, in summary, we will move to the Topping Off state if both of the */
	/* following are true: */
	/*  */
	/*  1) The maximum current set in the hardware is greater than the charging */
	/*     threshold. */
	/*  -AND- */
	/*  2) The actual current flow is also higher than the threshold (as */
	/*     indicated by the charge status bit). */
	/*  */
	/* -------------------------------------------------------------------------- */



		ddi_bc_hwSetCurrentThreshold(g_ddi_bc_Configuration.
					     u16ChargingThresholdCurrent);


	{
		uint16_t u16ActualProgrammedCurrent = ddi_bc_hwGetMaxCurrent();

		/* ---------------------------------------------------------------------- */
		/* Get the Maximum current that we will ramp to. */
		/* ---------------------------------------------------------------------- */

		/* ---------------------------------------------------------------------- */
		/* Not all possible values are expressible by the BATTCHRG_I bitfield. */
		/* The following coverts the max current value into the the closest hardware */
		/* expressible bitmask equivalent.  Then, it converts this back to the actual */
		/* decimal current value that this bitmask represents. */
		/* ---------------------------------------------------------------------- */

		uint16_t u16CurrentRampTarget = ddi_bc_RampGetTarget();

		if (u16CurrentRampTarget > ddi_bc_RampGetLimit())
			u16CurrentRampTarget = ddi_bc_RampGetLimit();

		/* ---------------------------------------------------------------------- */
		/* Not all possible values are expressible by the BATTCHRG_I bitfield. */
		/* The following coverts the max current value into the the closest hardware */
		/* expressible bitmask equivalent.  Then, it converts this back to the actual */
		/* decimal current value that this bitmask represents. */
		/* ---------------------------------------------------------------------- */

		u16CurrentRampTarget =
		    ddi_bc_hwExpressibleCurrent(u16CurrentRampTarget);

		/* ---------------------------------------------------------------------- */
		/* We want to wait before we check the charge status bit until the ramping */
		/* up is complete.  Because the charge status bit is noisy, we want to */
		/* disregard it until the programmed charge currint in BATTCHRG_I is well */
		/* beyond the STOP_ILIMIT value. */
		/* ---------------------------------------------------------------------- */
		if ((u16ActualProgrammedCurrent >= u16CurrentRampTarget) &&
		    !ddi_bc_hwGetChargeStatus()) {
			uint8_t u8IlimitThresholdLimit;
			/* ---------------------------------------------------------------------- */
			/* If control arrives here, the hardware flag is telling us that the */
			/* charging current has fallen below the threshold. We need to see this */
			/* happen twice consecutively before we believe it. Increment the count. */
			/* ---------------------------------------------------------------------- */

			iStatusCount++;


			u8IlimitThresholdLimit = 10;

			/* ---------------------------------------------------------------------- */
			/* How many times in a row have we seen this status bit low? */
			/* ---------------------------------------------------------------------- */

			if (iStatusCount >= u8IlimitThresholdLimit) {

				/*
				 * If control arrives here, we've seen the
				 * CHRGSTS bit low too many times. This means
				 * it's time to move to the Topping Off state.
				 * First, reset the status count for the next
				 * time we're in this state.
				 */

				iStatusCount = 0;

#ifdef CONFIG_POWER_SUPPLY_DEBUG
				u16ExternalBatteryPowerVoltageCheck =
				    ddi_bc_hwGetBatteryVoltage();
#endif



				/* Move to the Topping Off state */


				TransitionToToppingOff();

				/* ------------------------------------------------------------------ */
				/* Return success. */
				/* ------------------------------------------------------------------ */

				return DDI_BC_STATUS_SUCCESS;

			}

		} else {

			/* ---------------------------------------------------------------------- */
			/* If control arrives here, the battery is still charging. Clear the */
			/* status count. */
			/* ---------------------------------------------------------------------- */

			iStatusCount = 0;

		}

	}

	/* -------------------------------------------------------------------------- */
	/* Have we been in this state too long? */
	/* -------------------------------------------------------------------------- */

	if (g_ddi_bc_u32StateTimer >= g_ddi_bc_Configuration.u32ChargingTimeout) {

		/* ---------------------------------------------------------------------- */
		/* If control arrives here, we've been here too long. */
		/* ---------------------------------------------------------------------- */

		ddi_bc_gBrokenReason = DDI_BC_BROKEN_CHARGING_TIMEOUT;

		TransitionToBroken();

		/* ---------------------------------------------------------------------- */
		/* Tell our caller the battery appears to be broken. */
		/* ---------------------------------------------------------------------- */

		return DDI_BC_STATUS_BROKEN;

	}
	/* -------------------------------------------------------------------------- */
	/* If control arrives here, we're staying in this state. Step the current */
	/* ramp. */
	/* -------------------------------------------------------------------------- */

	ddi_bc_RampStep(g_ddi_bc_Configuration.u32StateMachinePeriod);

	/* -------------------------------------------------------------------------- */
	/* Return success. */
	/* -------------------------------------------------------------------------- */

	return DDI_BC_STATUS_SUCCESS;

}


/*  */
/* brief Topping Off state function. */
/*  */
/* fntype Function */
/*  */
/*  This function implements the Topping Off state. */
/*  */

static ddi_bc_Status_t ddi_bc_ToppingOff(void)
{

	/* -------------------------------------------------------------------------- */
	/* The first order of business is to update alarms. */

	/* -------------------------------------------------------------------------- */

	ddi_bc_RampUpdateAlarms();

	/* -------------------------------------------------------------------------- */
	/* Increment the state timer. Notice that, unlike other states, we increment */
	/* the state timer whether or not we're under an alarm. */
	/* -------------------------------------------------------------------------- */

	g_ddi_bc_u32StateTimer += g_ddi_bc_Configuration.u32StateMachinePeriod;

	/* -------------------------------------------------------------------------- */
	/* Check if the power supply is still around. */
	/* -------------------------------------------------------------------------- */

	if (!ddi_bc_hwPowerSupplyIsPresent()) {

		/* ---------------------------------------------------------------------- */
		/* If control arrives here, the power supply has been removed. Go back */
		/* and wait. */
		/* --------------------------------------------------------------------- */

		TransitionToWaitingToCharge();

		/* ---------------------------------------------------------------------- */
		/* Return success. */
		/* ---------------------------------------------------------------------- */

		return DDI_BC_STATUS_SUCCESS;

	}

	/* -------------------------------------------------------------------------- */
	/* Are we done topping off? */
	/* -------------------------------------------------------------------------- */
	if (g_ddi_bc_u32StateTimer >= g_ddi_bc_Configuration.u32TopOffPeriod) {

		/* ---------------------------------------------------------------------- */
		/* If control arrives here, we're done topping off. */
		/* ---------------------------------------------------------------------- */

		TransitionToWaitingToCharge();

	}
	/* -------------------------------------------------------------------------- */
	/* If control arrives here, we're staying in this state. Step the current */
	/* ramp. */
	/* -------------------------------------------------------------------------- */

	ddi_bc_RampStep(g_ddi_bc_Configuration.u32StateMachinePeriod);

	/* -------------------------------------------------------------------------- */
	/* Return success. */
	/* -------------------------------------------------------------------------- */

	return DDI_BC_STATUS_SUCCESS;

}


/* End of file */

/*  @} */
