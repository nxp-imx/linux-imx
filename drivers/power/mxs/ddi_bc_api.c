/*
 * Copyright (C) 2010 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */


/* Includes */


#include <linux/kernel.h>
#include "ddi_bc_internal.h"


/* Variables */


/* This structure holds the current Battery Charger configuration. */

ddi_bc_Cfg_t g_ddi_bc_Configuration;

extern uint32_t g_ddi_bc_u32StateTimer;
extern ddi_bc_BrokenReason_t ddi_bc_gBrokenReason;
extern bool bRestartChargeCycle;


/* Code */




/* brief Report the Battery Charger configuration. */

/* fntype Function */

/* This function reports the Battery Charger configuration. */

/* Note that, if the Battery Charger has not yet been initialized, the data */
/* returned by this function is unknown. */

/* param[in,out]  pCfg  A pointer to a structure that will receive the data. */


void ddi_bc_QueryCfg(ddi_bc_Cfg_t *pCfg)
{

	/* -------------------------------------------------------------------------- */
	/* Return the current configuration. */
	/* -------------------------------------------------------------------------- */

	*pCfg = g_ddi_bc_Configuration;

}



/* brief Shut down the Battery Charger. */

/* fntype Function */

/* This function immediately shuts down the Battery Charger hardware and */
/* returns the state machine to the Uninitialized state. Use this function to */
/* safely mummify the battery charger before retiring it from memory. */


void ddi_bc_ShutDown()
{

	/* -------------------------------------------------------------------------- */
	/* Reset the current ramp. */
	/* -------------------------------------------------------------------------- */

	ddi_bc_RampReset();

	/* -------------------------------------------------------------------------- */
	/* Move to the Uninitialized state. */
	/* -------------------------------------------------------------------------- */

	g_ddi_bc_State = DDI_BC_STATE_UNINITIALIZED;

}



/* brief Advances the state machine. */

/* fntype Function */

/* This function advances the state machine. */

/* retval DDI_BC_STATUS_SUCCESS          If all goes well */
/* retval DDI_BC_STATUS_NOT_INITIALIZED  If the Battery Charger is not yet */
/*                                        initialized. */
/* retval DDI_BC_STATUS_BROKEN           If the battery violated a time-out */
/*                                        and has been declared broken. */


ddi_bc_Status_t ddi_bc_StateMachine()
{
	int ret, state;

	/* -------------------------------------------------------------------------- */
	/* Check if we've been initialized yet. */
	/* -------------------------------------------------------------------------- */

	if (g_ddi_bc_State == DDI_BC_STATE_UNINITIALIZED) {
		return DDI_BC_STATUS_NOT_INITIALIZED;
	}
	/* -------------------------------------------------------------------------- */
	/* Execute the function for the current state. */
	/* -------------------------------------------------------------------------- */

	state = g_ddi_bc_State;
	ret = (stateFunctionTable[g_ddi_bc_State] ());
	if (state != g_ddi_bc_State)
		pr_debug("Charger: transit from state %d to %d\n",
			state, g_ddi_bc_State);
	return ret;

}



/* brief Get the Battery Charger's current state. */

/* fntype Function */

/* This function returns the current state. */

/* retval The current state. */


ddi_bc_State_t ddi_bc_GetState()
{
	/* -------------------------------------------------------------------------- */
	/* Return the current state. */
	/* -------------------------------------------------------------------------- */

	return g_ddi_bc_State;

}



/* brief Disable the Battery Charger. */

/* fntype Function */

/* This function forces the Battery Charger into the Disabled state. */

/* retval DDI_BC_STATUS_SUCCESS          If all goes well */
/* retval DDI_BC_STATUS_NOT_INITIALIZED  If the Battery Charger is not yet */
/*                                        initialized. */


ddi_bc_Status_t ddi_bc_SetDisable()
{

	/* -------------------------------------------------------------------------- */
	/* Check if we've been initialized yet. */
	/* -------------------------------------------------------------------------- */

	if (g_ddi_bc_State == DDI_BC_STATE_UNINITIALIZED) {
		return DDI_BC_STATUS_NOT_INITIALIZED;
	}
	/* -------------------------------------------------------------------------- */
	/* Check if we've been initialized yet. */
	/* -------------------------------------------------------------------------- */

	if (g_ddi_bc_State == DDI_BC_STATE_BROKEN) {
		return DDI_BC_STATUS_BROKEN;
	}
	/* -------------------------------------------------------------------------- */
	/* Reset the current ramp. This will jam the current to zero and power off */
	/* the charging hardware. */
	/* -------------------------------------------------------------------------- */

	ddi_bc_RampReset();

	/* -------------------------------------------------------------------------- */
	/* Reset the state timer. */
	/* -------------------------------------------------------------------------- */

	g_ddi_bc_u32StateTimer = 0;

	/* -------------------------------------------------------------------------- */
	/* Move to the Disabled state. */
	/* -------------------------------------------------------------------------- */

	g_ddi_bc_State = DDI_BC_STATE_DISABLED;

	/* -------------------------------------------------------------------------- */
	/* Return success. */
	/* -------------------------------------------------------------------------- */

	return DDI_BC_STATUS_SUCCESS;

}



/* brief Enable the Battery Charger. */

/* fntype Function */

/* If the Battery Charger is in the Disabled state, this function moves it to */
/* the Waiting to Charge state. */

/* retval DDI_BC_STATUS_SUCCESS          If all goes well */
/* retval DDI_BC_STATUS_NOT_INITIALIZED  If the Battery Charger is not yet */
/*                                        initialized. */
/* retval DDI_BC_STATUS_NOT_DISABLED     If the Battery Charger is not */
/*                                        disabled. */


ddi_bc_Status_t ddi_bc_SetEnable()
{

	/* -------------------------------------------------------------------------- */
	/* Check if we've been initialized yet. */
	/* -------------------------------------------------------------------------- */

	if (g_ddi_bc_State == DDI_BC_STATE_UNINITIALIZED) {
		return DDI_BC_STATUS_NOT_INITIALIZED;
	}
	/* -------------------------------------------------------------------------- */
	/* If we're not in the Disabled state, this is pointless. */
	/* -------------------------------------------------------------------------- */

	if (g_ddi_bc_State != DDI_BC_STATE_DISABLED) {
		return DDI_BC_STATUS_NOT_DISABLED;
	}
	/* -------------------------------------------------------------------------- */
	/* Reset the state timer. */
	/* -------------------------------------------------------------------------- */

	g_ddi_bc_u32StateTimer = 0;
	/* -------------------------------------------------------------------------- */
	/* Move to the Waiting to Charge state. */
	/* -------------------------------------------------------------------------- */

	g_ddi_bc_State = DDI_BC_STATE_WAITING_TO_CHARGE;

	/* -------------------------------------------------------------------------- */
	/* Return success. */
	/* -------------------------------------------------------------------------- */

	return DDI_BC_STATUS_SUCCESS;

}



/* brief Declare the battery to be broken. */

/* fntype Function */

/* This function forces the Battery Charger into the Broken state. */

/* retval DDI_BC_STATUS_SUCCESS          If all goes well */
/* retval DDI_BC_STATUS_NOT_INITIALIZED  If the Battery Charger is not yet */
/*                                        initialized. */


ddi_bc_Status_t ddi_bc_SetBroken()
{

	/* -------------------------------------------------------------------------- */
	/* Check if we've been initialized yet. */
	/* -------------------------------------------------------------------------- */

	if (g_ddi_bc_State == DDI_BC_STATE_UNINITIALIZED) {
		return DDI_BC_STATUS_NOT_INITIALIZED;
	}
	/* -------------------------------------------------------------------------- */
	/* Reset the current ramp. This will jam the current to zero and power off */
	/* the charging hardware. */
	/* -------------------------------------------------------------------------- */

	ddi_bc_RampReset();

	/* -------------------------------------------------------------------------- */
	/* Reset the state timer. */
	/* -------------------------------------------------------------------------- */

	g_ddi_bc_u32StateTimer = 0;

	/* -------------------------------------------------------------------------- */
	/* Move to the Broken state. */
	/* -------------------------------------------------------------------------- */

	ddi_bc_gBrokenReason = DDI_BC_BROKEN_CHARGING_TIMEOUT;

	g_ddi_bc_State = DDI_BC_STATE_BROKEN;

	/* -------------------------------------------------------------------------- */
	/* Return success. */
	/* -------------------------------------------------------------------------- */

	return DDI_BC_STATUS_SUCCESS;

}



/* brief Declare the battery to be fixed. */

/* fntype Function */

/* If the Battery Charger is in the Broken state, this function moves it to */
/* the Disabled state. */

/* retval DDI_BC_STATUS_SUCCESS          If all goes well */
/* retval DDI_BC_STATUS_NOT_INITIALIZED  If the Battery Charger is not yet */
/*                                        initialized. */
/* retval DDI_BC_STATUS_NOT_BROKEN       If the Battery Charger is not broken. */


ddi_bc_Status_t ddi_bc_SetFixed()
{

	/* -------------------------------------------------------------------------- */
	/* Check if we've been initialized yet. */
	/* -------------------------------------------------------------------------- */

	if (g_ddi_bc_State == DDI_BC_STATE_UNINITIALIZED) {
		return DDI_BC_STATUS_NOT_INITIALIZED;
	}
	/* -------------------------------------------------------------------------- */
	/* If we're not in the Broken state, this is pointless. */
	/* -------------------------------------------------------------------------- */

	if (g_ddi_bc_State != DDI_BC_STATE_BROKEN) {
		return DDI_BC_STATUS_NOT_BROKEN;
	}
	/* -------------------------------------------------------------------------- */
	/* Reset the state timer. */
	/* -------------------------------------------------------------------------- */

	g_ddi_bc_u32StateTimer = 0;

	/* -------------------------------------------------------------------------- */
	/* Unitialize the Broken Reason */
	/* -------------------------------------------------------------------------- */

	ddi_bc_gBrokenReason = DDI_BC_BROKEN_UNINITIALIZED;

	/* -------------------------------------------------------------------------- */
	/* Move to the Disabled state. */
	/* -------------------------------------------------------------------------- */

	g_ddi_bc_State = DDI_BC_STATE_DISABLED;

	/* -------------------------------------------------------------------------- */
	/* Return success. */
	/* -------------------------------------------------------------------------- */

	return DDI_BC_STATUS_SUCCESS;

}



/* brief Set the current limit. */

/* fntype Function */

/* This function applies a limit to the current that the Battery Charger can */
/* draw. */

/* param[in]  u16Limit  The maximum current the Battery Charger can draw */
/*                       (in mA). */

/* retval  The expressible version of the limit. */


uint16_t ddi_bc_SetCurrentLimit(uint16_t u16Limit)
{

	/* -------------------------------------------------------------------------- */
	/* Set the limit and return what is actually expressible. */
	/* -------------------------------------------------------------------------- */

	return ddi_bc_RampSetLimit(u16Limit);

}



/* brief Report the current limit. */

/* fntype Function */

/* This function reports the limit to the current that the Battery Charger can */
/* draw. */

/* retval  The current limit. */


uint16_t ddi_bc_GetCurrentLimit(void)
{

	/* -------------------------------------------------------------------------- */
	/* Set the limit and return what is actually expressible. */
	/* -------------------------------------------------------------------------- */

	return ddi_bc_RampGetLimit();

}



/* brief Set the battery charger state machine period. */

/* fntype Function */

/* This function sets a new state machine period.  The Period and Slope should */
/* be coordinated to achieve the minimal ramp step current which will minimize */
/* transients on the system. */

/* param[in]  u32StateMachinePeriod  (in milliseconds) */
/* param[in]  u16CurrentRampSlope (in mA/s) */

/* retval SUCCESS                        If all goes well */
/* retval ERROR_DDI_BCM_NOT_INITIALIZED  If the Battery Charger is not yet */
/*                                        initialized. */


ddi_bc_Status_t ddi_bc_SetNewPeriodAndSlope(uint32_t u32StateMachinePeriod,
					    uint16_t u16CurrentRampSlope)
{
	/* -------------------------------------------------------------------------- */
	/* Check if we've been initialized yet. */
	/* -------------------------------------------------------------------------- */
	bool bDisableRequired;

	if (g_ddi_bc_State == DDI_BC_STATE_UNINITIALIZED) {
		return DDI_BC_STATUS_NOT_INITIALIZED;
	}

	if (g_ddi_bc_State == DDI_BC_STATE_DISABLED)
		bDisableRequired = false;
	else {
		bDisableRequired = true;
		ddi_bc_SetDisable();
	}

	/* Looking at the code, changing the period while the battery charger is running */
	/* doesn't seem to have a negative affect.  One could wrap this in the mutex */
	/* or implement further coordination if it did. */
	g_ddi_bc_Configuration.u32StateMachinePeriod = u32StateMachinePeriod;
	g_ddi_bc_Configuration.u16CurrentRampSlope = u16CurrentRampSlope;

	if (bDisableRequired)
		ddi_bc_SetEnable();

	return DDI_BC_STATUS_SUCCESS;

}



/* brief Report the state machine period. */

/* fntype Function */

/* This function reports the battery charger period. */

/* retval  The battery charger period (in milliseconds). */


uint32_t ddi_bc_GetStateMachinePeriod()
{
	return g_ddi_bc_Configuration.u32StateMachinePeriod;
}



/* brief Report the current ramp slope. */

/* fntype Function */

/* This function reports the current ramp slope. */

/* retval  The current ramp slope (in mA/s). */


uint32_t ddi_bc_GetCurrentRampSlope()
{
	return g_ddi_bc_Configuration.u16CurrentRampSlope;
}



/* brief Report the time spent in the present state (milliseconds) */

/* fntype Function */

/* This function reports the time spent in the present charging state.  Note that */
/* for the states that actually charge the battery, this time does not include the */
/* time spent under alarm conditions such as die termperature alarm or battery */
/* temperature alarm. */

/* retval  The time spent in the current state in milliseconds. */


uint32_t ddi_bc_GetStateTime(void)
{
	return g_ddi_bc_u32StateTimer;
}



/* brief Report the reason for being in the broken state */

/* fntype Function */


/* retval  ddi_bc_BrokenReason_t enumeration */


ddi_bc_BrokenReason_t ddi_bc_GetBrokenReason(void)
{
	return ddi_bc_gBrokenReason;
}



/* brief Restart the charge cycle */

/* fntype Function */


/* retval  SUCCESS */


ddi_bc_Status_t ddi_bc_ForceChargingToStart(void)
{
	static int16_t restarts;

	if (restarts < DDI_BC_MAX_RESTART_CYCLES) {
		restarts++;
		bRestartChargeCycle = true;
	}

	return DDI_BC_STATUS_SUCCESS;
}


/* End of file */

/* @} */
