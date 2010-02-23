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

#include "ddi_bc_internal.h"


/* addtogroup ddi_bc */
/*  @{ */
/*  */
/* Copyright (c) 2004-2005 SigmaTel, Inc. */
/*  */
/* file       ddi_bc_init.c */
/* brief      Contains the Battery Charger initialization function. */
/* date       06/2005 */
/*  */
/*  This file contains Battery Charger initialization function. */
/*  */



/* Includes and external references */

#include <mach/ddi_bc.h>
#include "ddi_bc_internal.h"


/* Code */



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

ddi_bc_Status_t ddi_bc_Init(ddi_bc_Cfg_t *pCfg)
{

	/* -------------------------------------------------------------------------- */
	/* We can only be initialized if we're in the Uninitialized state. */
	/* -------------------------------------------------------------------------- */

	if (g_ddi_bc_State != DDI_BC_STATE_UNINITIALIZED) {
		return DDI_BC_STATUS_ALREADY_INITIALIZED;
	}
	/* -------------------------------------------------------------------------- */
	/* Check if the battery charger hardware has been disabled by laser fuse. */
	/* -------------------------------------------------------------------------- */

	if (!ddi_power_GetBatteryChargerEnabled())
		return DDI_BC_STATUS_HARDWARE_DISABLED;

	/* -------------------------------------------------------------------------- */
	/* Check if the power supply has been set up for a non-rechargeable battery. */
	/* -------------------------------------------------------------------------- */

	switch (ddi_power_GetBatteryMode()) {

	case DDI_POWER_BATT_MODE_LIION:
		break;

		/* TODO: we'll need to do NiMH also */
	default:
		return DDI_BC_STATUS_BAD_BATTERY_MODE;
		/* break; */

	}

	/* -------------------------------------------------------------------------- */
	/* Make sure that the clock gate has been opened for the power supply */
	/* registers. If not, then none of our writes to those registers will */
	/* succeed, which will kind of slow us down... */
	/* -------------------------------------------------------------------------- */

	if (ddi_power_GetPowerClkGate()) {
		return DDI_BC_STATUS_CLOCK_GATE_CLOSED;
	}
	/* -------------------------------------------------------------------------- */
	/* Check the incoming configuration for nonsense. */
	/* -------------------------------------------------------------------------- */

	/*  */
	/* Only permitted charging voltage: 4200mV. */
	/*  */

	if (pCfg->u16ChargingVoltage != DDI_BC_LIION_CHARGING_VOLTAGE) {
		return DDI_BC_STATUS_CFG_BAD_CHARGING_VOLTAGE;
	}
	/*  */
	/* There are 8 LRADC channels. */
	/*  */

	if (pCfg->u8BatteryTempChannel > 7) {
		return DDI_BC_STATUS_CFG_BAD_BATTERY_TEMP_CHANNEL;
	}
	/* -------------------------------------------------------------------------- */
	/* Accept the configuration. */
	/* -------------------------------------------------------------------------- */

	/* -------------------------------------------------------------------------- */
	/*  ddi_bc_Cfg_t.u16ChargingThresholdCurrent is destined for the */
	/*  register field HW_POWER_BATTCHRG.STOP_ILIMIT.  This 4-bit field */
	/*  is unevenly quantized to provide a useful range of currents.  A */
	/*  side effect of the quantization is that the field can only be */
	/*  set to certain unevenly-spaced values. */
	/*  */
	/*  Here, we use the two functions that manipulate the register field */
	/*  to adjust u16ChargingThresholdCurrent to match the quantized value. */
	/* -------------------------------------------------------------------------- */
	pCfg->u16ChargingThresholdCurrent =
	    ddi_power_ExpressibleCurrent(pCfg->u16ChargingThresholdCurrent);

	/* -------------------------------------------------------------------------- */
	/*  ...similar situation with ddi_bc_Cfg_t.u16BatteryTempSafeCurrent and */
	/*  u16DieTempSafeCurrent. */
	/* -------------------------------------------------------------------------- */
	pCfg->u16BatteryTempSafeCurrent =
	    ddi_power_ExpressibleCurrent(pCfg->u16BatteryTempSafeCurrent);
	pCfg->u16DieTempSafeCurrent =
	    ddi_power_ExpressibleCurrent(pCfg->u16DieTempSafeCurrent);

	g_ddi_bc_Configuration = *pCfg;

	/* -------------------------------------------------------------------------- */
	/* Turn the charger hardware off. This is a very important initial condition */
	/* because we only flip the power switch on the hardware when we make */
	/* transitions. Baseline, it needs to be off. */
	/* -------------------------------------------------------------------------- */

	ddi_power_SetChargerPowered(0);

	/* -------------------------------------------------------------------------- */
	/* Reset the current ramp. This will jam the current to zero and power off */
	/* the charging hardware. */
	/* -------------------------------------------------------------------------- */

	ddi_bc_RampReset();

	/* -------------------------------------------------------------------------- */
	/* Move to the Disabled state. */
	/* -------------------------------------------------------------------------- */

	g_ddi_bc_State = DDI_BC_STATE_DISABLED;

	/* -------------------------------------------------------------------------- */
	/* Return success. */
	/* -------------------------------------------------------------------------- */
#ifdef CONFIG_POWER_SUPPLY_DEBUG
	printk("%s: success\n", __func__);
#endif
	return DDI_BC_STATUS_SUCCESS;

}


/* End of file */

/*  @} */
