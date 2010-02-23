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


/* addtogroup ddi_bc */
/*  @{ */
/*  */
/* Copyright (c) 2004-2005 SigmaTel, Inc. */
/*  */
/* file       ddi_bc_ramp.c */
/* brief      Contains the Battery Charger current ramp controller. */
/* date       06/2005 */
/*  */
/*  This file contains Battery Charger current ramp controller. */
/*  */



/* Includes and external references */


#include <mach/ddi_bc.h>
#include "ddi_bc_internal.h"


/* Definitions */


/*  This is the control structure for the current ramp. */

typedef struct _ddi_bc_RampControl {

	uint32_t u32AccumulatedTime;

	/* < The accumulated time since we last changed the actual */
	/* < current setting in the hardware. If the time between */
	/* < steps is quite short, we may have to wait for several steps */
	/* < before we can actually change the hardware setting. */

	uint16_t u16Target;

	/* < The target current, regardless of expressibility. */

	uint16_t u16Limit;

	/* < The current limit, regardless of expressibility. */

	uint8_t dieTempAlarm:1;

	/* < Indicates if we are operating under a die temperature */
	/* < alarm. */

	uint8_t batteryTempAlarm:1;

	/* < Indicates if we are operating under a battery temperature */
	/* < alarm. */

	uint8_t ambientTempAlarm:1;

	/* < Indicates if we are operating under an ambient temperature */
	/* < alarm. */

} ddi_bc_RampControl_t;


/* Variables */


/*  This structure contains control information for the current ramp. */

static ddi_bc_RampControl_t g_RampControl;


/* Code */



/*  */
/* brief Reset the current ramp. */
/*  */
/* fntype Function */
/*  */
/*  This function resets the current ramp. */
/*  */
/*  Note that this function does NOT reset the temperature alarms or the current */
/*  limit. Those can only be changed explicitly. */
/*  */

void ddi_bc_RampReset()
{

	/* -------------------------------------------------------------------------- */
	/* Reset the control structure. */
	/* -------------------------------------------------------------------------- */

	g_RampControl.u32AccumulatedTime = 0;
	g_RampControl.u16Target = 0;

	/* -------------------------------------------------------------------------- */
	/* Step the ramp. Note that we don't care if this function returns an error. */
	/* We're stepping the ramp to make sure it takes immediate effect, if */
	/* possible. But, for example, if the Battery Charger is not yet */
	/* initialized, it doesn't matter. */
	/* -------------------------------------------------------------------------- */

	ddi_bc_RampStep(0);

}


/*  */
/* brief Set the target current. */
/*  */
/* fntype Function */
/*  */
/*  This function sets the target current and implements it immediately. */
/*  */
/*  Note that this function does NOT reset the temperature alarms. Those can */
/*  only be reset explicitly. */
/*  */
/* param[in]  u16Target  The target current. */
/*  */
/* retval  The expressible version of the target. */
/*  */

uint16_t ddi_bc_RampSetTarget(uint16_t u16Target)
{

	/* -------------------------------------------------------------------------- */
	/* Set the target. */
	/* -------------------------------------------------------------------------- */

	g_RampControl.u16Target = u16Target;

	/* -------------------------------------------------------------------------- */
	/* Step the ramp. Note that we don't care if this function returns an error. */
	/* We're stepping the ramp to make sure it takes immediate effect, if */
	/* possible. But, for example, if the Battery Charger is not yet */
	/* initialized, it doesn't matter. */
	/* -------------------------------------------------------------------------- */

	ddi_bc_RampStep(0);

	/* -------------------------------------------------------------------------- */
	/* Compute and return the expressible target. */
	/* -------------------------------------------------------------------------- */

	return ddi_bc_hwExpressibleCurrent(u16Target);

}


/*  */
/* brief Report the target. */
/*  */
/* fntype Function */
/*  */
/*  This function reports the target. */
/*  */
/* retval  The target. */
/*  */

uint16_t ddi_bc_RampGetTarget(void)
{

	/* -------------------------------------------------------------------------- */
	/* Return the target. */
	/* -------------------------------------------------------------------------- */

	return g_RampControl.u16Target;

}


/*  */
/* brief Set the current limit. */
/*  */
/* fntype Function */
/*  */
/*  This function sets the current limit and implements it immediately. */
/*  */
/* param[in]  u16Limit  The current limit. */
/*  */
/* retval  The expressible version of the limit. */
/*  */

uint16_t ddi_bc_RampSetLimit(uint16_t u16Limit)
{

	/* -------------------------------------------------------------------------- */
	/* Set the limit. */
	/* -------------------------------------------------------------------------- */

	g_RampControl.u16Limit = u16Limit;

	/* -------------------------------------------------------------------------- */
	/* Step the ramp. Note that we don't care if this function returns an error. */
	/* We're stepping the ramp to make sure it takes immediate effect, if */
	/* possible. But, for example, if the Battery Charger is not yet */
	/* initialized, it doesn't matter. */
	/* -------------------------------------------------------------------------- */

	ddi_bc_RampStep(0);

	/* -------------------------------------------------------------------------- */
	/* Compute and return the expressible limit. */
	/* -------------------------------------------------------------------------- */

	return ddi_bc_hwExpressibleCurrent(u16Limit);

}


/*  */
/* brief Report the current limit. */
/*  */
/* fntype Function */
/*  */
/*  This function reports the current limit. */
/*  */
/* retval  The current limit. */
/*  */

uint16_t ddi_bc_RampGetLimit(void)
{

	/* -------------------------------------------------------------------------- */
	/* Return the current limit. */
	/* -------------------------------------------------------------------------- */

	return g_RampControl.u16Limit;

}


/*  */
/* brief Update alarms. */
/*  */
/* fntype Function */
/*  */
/*  This function checks for all alarms and updates the current ramp */
/*  accordingly. */
/*  */

void ddi_bc_RampUpdateAlarms()
{

	/* Set to true if something changed and we need to step the ramp right away. */

	int iStepTheRamp = 0;

	/* -------------------------------------------------------------------------- */
	/* Are we monitoring die temperature? */
	/* -------------------------------------------------------------------------- */

	if (g_ddi_bc_Configuration.monitorDieTemp) {

		/* ---------------------------------------------------------------------- */
		/* Get the die temperature range. */
		/* ---------------------------------------------------------------------- */

		int16_t i16Low;
		int16_t i16High;

		ddi_bc_hwGetDieTemp(&i16Low, &i16High);

		/* ---------------------------------------------------------------------- */
		/* Now we need to decide if it's time to raise or lower the alarm. The */
		/* first question to ask is: Were we already under an alarm? */
		/* ---------------------------------------------------------------------- */

		if (g_RampControl.dieTempAlarm) {

			/* ------------------------------------------------------------------ */
			/* If control arrives here, we were already under an alarm. We'll */
			/* change that if the high end of the temperature range drops below */
			/* the low temperature mark. */
			/* ------------------------------------------------------------------ */

			if (i16High < g_ddi_bc_Configuration.u8DieTempLow) {

				/* -------------------------------------------------------------- */
				/* If control arrives here, we're safe now. Drop the alarm. */
				/* -------------------------------------------------------------- */

				g_RampControl.dieTempAlarm = 0;

				iStepTheRamp = !0;

#ifdef CONFIG_POWER_SUPPLY_DEBUG
				printk("Battery charger: releasing "
				       "die temp alarm: [%d, %d] < %d\r\n",
				       (int32_t) i16Low, (int32_t) i16High,
				       (int32_t) g_ddi_bc_Configuration.
				       u8DieTempLow);
#endif

			}

		} else {

			/* ------------------------------------------------------------------ */
			/* If control arrives here, we were not under an alarm. We'll change */
			/* that if the high end of the temperature range rises above the */
			/* high temperature mark. */
			/* ------------------------------------------------------------------ */

			if (i16High >= g_ddi_bc_Configuration.u8DieTempHigh) {

				/* -------------------------------------------------------------- */
				/* If control arrives here, we're running too hot. Raise the */
				/* alarm. */
				/* -------------------------------------------------------------- */

				g_RampControl.dieTempAlarm = 1;

				iStepTheRamp = !0;

#ifdef CONFIG_POWER_SUPPLY_DEBUG
				printk("Battery charger: declaring "
				       "die temp alarm: [%d, %d] >= %d\r\n",
				       (int32_t) i16Low, (int32_t) i16High,
				       (int32_t) g_ddi_bc_Configuration.
				       u8DieTempLow);
#endif
			}

		}

	}
	/* -------------------------------------------------------------------------- */
	/* Are we monitoring battery temperature? */
	/* -------------------------------------------------------------------------- */

	if (g_ddi_bc_Configuration.monitorBatteryTemp) {

		ddi_bc_Status_t status;

		/* ---------------------------------------------------------------------- */
		/* Get the battery temperature reading. */
		/* ---------------------------------------------------------------------- */

		uint16_t u16Reading;

		status = ddi_bc_hwGetBatteryTemp(&u16Reading);

		/* ---------------------------------------------------------------------- */
		/* If there was a problem, then we ignore the reading. Otherwise, let's */
		/* have a look. */
		/* ---------------------------------------------------------------------- */

		if (status == DDI_BC_STATUS_SUCCESS) {

			/* ------------------------------------------------------------------ */
			/* Now we need to decide if it's time to raise or lower the alarm. */
			/* The first question to ask is: Were we already under an alarm? */
			/* ------------------------------------------------------------------ */

			if (g_RampControl.batteryTempAlarm) {

				/* -------------------------------------------------------------- */
				/* If control arrives here, we were already under an alarm. */
				/* We'll change that if the reading drops below the low mark. */
				/* -------------------------------------------------------------- */

				if (u16Reading <
				    g_ddi_bc_Configuration.u16BatteryTempLow) {

					/* ---------------------------------------------------------- */
					/* If control arrives here, we're safe now. Drop the alarm. */
					/* ---------------------------------------------------------- */

					g_RampControl.batteryTempAlarm = 0;

					iStepTheRamp = !0;

				}

			} else {

				/* -------------------------------------------------------------- */
				/* If control arrives here, we were not under an alarm. We'll */
				/* change that if the reading rises above the high mark. */
				/* -------------------------------------------------------------- */

				if (u16Reading >=
				    g_ddi_bc_Configuration.u16BatteryTempHigh) {

					/* ---------------------------------------------------------- */
					/* If control arrives here, we're running too hot. Raise the */
					/* alarm. */
					/* ---------------------------------------------------------- */

					g_RampControl.batteryTempAlarm = 1;

					iStepTheRamp = !0;

				}

			}

		}

	}
	/* -------------------------------------------------------------------------- */
	/* Do we need to step the ramp? */
	/* -------------------------------------------------------------------------- */

	if (iStepTheRamp)
		ddi_bc_RampStep(0);

}


/*  */
/* brief Reports the state of the die temperature alarm. */
/*  */
/* fntype Function */
/*  */
/*  This function reports the state of the die temperature alarm. */
/*  */
/* retval  The state of the die temperature alarm. */
/*  */

int ddi_bc_RampGetDieTempAlarm(void)
{
	return g_RampControl.dieTempAlarm;
}


/*  */
/* brief Reports the state of the battery temperature alarm. */
/*  */
/* fntype Function */
/*  */
/*  This function reports the state of the battery temperature alarm. */
/*  */
/* retval  The state of the battery temperature alarm. */
/*  */

int ddi_bc_RampGetBatteryTempAlarm(void)
{
	return g_RampControl.batteryTempAlarm;
}


/*  */
/* brief Reports the state of the ambient temperature alarm. */
/*  */
/* fntype Function */
/*  */
/*  This function reports the state of the ambient temperature alarm. */
/*  */
/* retval  The state of the ambient temperature alarm. */
/*  */

int ddi_bc_RampGetAmbientTempAlarm(void)
{
	return g_RampControl.ambientTempAlarm;
}


/*  */
/* brief Step the current ramp. */
/*  */
/* fntype Function */
/*  */
/*  This function steps the current ramp forward through the given amount of time. */
/*  */
/* param[in] u32Time  The time increment to add. */
/*  */
/* retval  DDI_BC_STATUS_SUCCESS          If the operation succeeded. */
/* retval  DDI_BC_STATUS_NOT_INITIALIZED  If the Battery Charger is not yet */
/*                                          initialized. */
/*  */

ddi_bc_Status_t ddi_bc_RampStep(uint32_t u32Time)
{

	uint16_t u16MaxNow;
	uint16_t u16Target;
	uint16_t u16Cart;
	int32_t i32Delta;

	/* -------------------------------------------------------------------------- */
	/* Make sure the Battery Charger is initialized. */
	/* -------------------------------------------------------------------------- */

	if (g_ddi_bc_State == DDI_BC_STATE_UNINITIALIZED) {
		return DDI_BC_STATUS_NOT_INITIALIZED;
	}
	/* -------------------------------------------------------------------------- */
	/* Figure out how much current the hardware is set to draw right now. */
	/* -------------------------------------------------------------------------- */

	u16MaxNow = ddi_bc_hwGetMaxCurrent();

	/* -------------------------------------------------------------------------- */
	/* Start with the target. */
	/* -------------------------------------------------------------------------- */

	u16Target = g_RampControl.u16Target;

	/* -------------------------------------------------------------------------- */
	/* Check the target against the hard limit. */
	/* -------------------------------------------------------------------------- */

	if (u16Target > g_RampControl.u16Limit)
		u16Target = g_RampControl.u16Limit;

	/* -------------------------------------------------------------------------- */
	/* Check if the die temperature alarm is active. */
	/* -------------------------------------------------------------------------- */

	if (g_RampControl.dieTempAlarm) {

		/* ---------------------------------------------------------------------- */
		/* If control arrives here, we are under a die temperature alarm. Clamp */
		/* the target current. */
		/* ---------------------------------------------------------------------- */

		if (u16Target > g_ddi_bc_Configuration.u16DieTempSafeCurrent) {
			u16Target =
			    g_ddi_bc_Configuration.u16DieTempSafeCurrent;
		}

	}
	/* -------------------------------------------------------------------------- */
	/* Check if the battery temperature alarm is active. */
	/* -------------------------------------------------------------------------- */

	if (g_RampControl.batteryTempAlarm) {

		/* ---------------------------------------------------------------------- */
		/* If control arrives here, we are under a battery temperature alarm. */
		/* Clamp the target current. */
		/* ---------------------------------------------------------------------- */

		if (u16Target >
		    g_ddi_bc_Configuration.u16BatteryTempSafeCurrent) {
			u16Target =
			    g_ddi_bc_Configuration.u16BatteryTempSafeCurrent;
		}

	}
	/* -------------------------------------------------------------------------- */
	/* Now we know the target current. Figure out what is actually expressible */
	/* in the hardware. */
	/* -------------------------------------------------------------------------- */

	u16Target = ddi_bc_hwExpressibleCurrent(u16Target);

	/* -------------------------------------------------------------------------- */
	/* Compute the difference between the expressible target and what's actually */
	/* set in the hardware right now. */
	/* -------------------------------------------------------------------------- */

	i32Delta = ((int32_t) u16Target) - ((int32_t) u16MaxNow);

	/* -------------------------------------------------------------------------- */
	/* Check if the delta is zero. */
	/* -------------------------------------------------------------------------- */

	if (i32Delta == 0) {

		/* ---------------------------------------------------------------------- */
		/* If control arrives here, there is no difference between what we want */
		/* and what's set in the hardware. */
		/*  */
		/* Before we leave, though, we don't want to leave any accumulated time */
		/* laying around for the next ramp up. Zero it out. */
		/* ---------------------------------------------------------------------- */

		g_RampControl.u32AccumulatedTime = 0;

		/* ---------------------------------------------------------------------- */
		/* Return success. */
		/* ---------------------------------------------------------------------- */

		return DDI_BC_STATUS_SUCCESS;

	}
	/* -------------------------------------------------------------------------- */
	/* Check if the delta is negative. */
	/* -------------------------------------------------------------------------- */

	if (i32Delta < 0) {

		/* ---------------------------------------------------------------------- */
		/* If control arrives here, the new target is lower than what's */
		/* currently set in the hardware. Since that means we're *reducing* the */
		/* current draw, we can do it right now. Just gimme a sec here... */
		/* ---------------------------------------------------------------------- */

		ddi_bc_hwSetMaxCurrent(u16Target);

#ifdef CONFIG_POWER_SUPPLY_DEBUG
		printk("Battery charger: setting max charge "
		       "current to: %hdmA\r\n", u16Target);
#endif

		/* ---------------------------------------------------------------------- */
		/* Flip the power switch on the charging hardware according to the new */
		/* current setting. */
		/* ---------------------------------------------------------------------- */

		ddi_bc_hwSetChargerPower(u16Target != 0);

		/* ---------------------------------------------------------------------- */
		/* We don't want to leave any accumulated time laying around for the */
		/* next ramp up. Zero it out. */
		/* ---------------------------------------------------------------------- */

		g_RampControl.u32AccumulatedTime = 0;

		/* ---------------------------------------------------------------------- */
		/* Return success. */
		/* ---------------------------------------------------------------------- */

		return DDI_BC_STATUS_SUCCESS;

	}
	/* -------------------------------------------------------------------------- */
	/* If control arrives here, the target current is higher than what's set in */
	/* the hardware right now. That means we're going to ramp it up. To do that, */
	/* we're going to "buy" more milliamps by "spending" milliseconds of time. */
	/* Add the time we've "banked" to the time we've been credited in this call. */
	/* -------------------------------------------------------------------------- */

	u32Time += g_RampControl.u32AccumulatedTime;

	/* -------------------------------------------------------------------------- */
	/* Now we know how much we can spend. How much current will it buy? */
	/* -------------------------------------------------------------------------- */

	u16Cart = (g_ddi_bc_Configuration.u16CurrentRampSlope * u32Time) / 1000;

	/* -------------------------------------------------------------------------- */
	/* Check how the current we can afford stacks up against the target we want. */
	/* -------------------------------------------------------------------------- */

	if ((u16MaxNow + u16Cart) < u16Target) {

		/* ---------------------------------------------------------------------- */
		/* If control arrives here, we can't afford to buy all the current we */
		/* want. Compute the maximum we can afford, and then figure out what we */
		/* can actually express in the hardware. */
		/* ---------------------------------------------------------------------- */

		u16Target = ddi_bc_hwExpressibleCurrent(u16MaxNow + u16Cart);

		/* ---------------------------------------------------------------------- */
		/* Check if the result isn't actually different from what's set in the */
		/* the hardware right now. */
		/* ---------------------------------------------------------------------- */

		if (u16Target == u16MaxNow) {

			/* ------------------------------------------------------------------ */
			/* If control arrives here, we are so poor that we can't yet afford */
			/* to buy enough current to make a change in the expressible */
			/* hardware setting. Since we didn't spend any of our time, put the */
			/* new balance back in the bank. */
			/* ------------------------------------------------------------------ */

			g_RampControl.u32AccumulatedTime = u32Time;

			/* ------------------------------------------------------------------ */
			/* Leave dispiritedly. */
			/* ------------------------------------------------------------------ */

			return DDI_BC_STATUS_SUCCESS;

		}

	}
	/* -------------------------------------------------------------------------- */
	/* If control arrives here, we can afford to buy enough current to get us */
	/* all the way to the target. Set it. */
	/* -------------------------------------------------------------------------- */

	ddi_bc_hwSetMaxCurrent(u16Target);

#ifdef CONFIG_POWER_SUPPLY_DEBUG
	printk("Battery charger: setting max charge"
	       "current to: %hdmA\r\n", u16Target);
#endif

	/* -------------------------------------------------------------------------- */
	/* Flip the power switch on the charging hardware according to the new */
	/* current setting. */
	/* -------------------------------------------------------------------------- */

	ddi_bc_hwSetChargerPower(u16Target != 0);

	/* -------------------------------------------------------------------------- */
	/* We're at the target, so we're finished buying current. Zero out the */
	/* account. */
	/* -------------------------------------------------------------------------- */

	g_RampControl.u32AccumulatedTime = 0;

	/* -------------------------------------------------------------------------- */
	/* Return success. */
	/* -------------------------------------------------------------------------- */

	return DDI_BC_STATUS_SUCCESS;

}


/* End of file */

/*  @} */
