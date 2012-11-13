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

#include "ddi_bc_internal.h"


/* Includes and external references */



/* Variables */



/* Code */



/*  */
/* brief Report if the battery charging hardware is available. */
/*  */
/* fntype Function */
/*  */
/*  This function reports if the battery charging hardware is available by */
/*  reading the corresponding laser fuse bit. */
/*  */
/* retval  Zero if the battery charging hardware is not available. Non-zero */
/*           otherwise. */
/*  */

int ddi_bc_hwBatteryChargerIsEnabled(void)
{
	/* TODO: replace ddi_bc_hwBatteryChargerIsEnabled with the function below in the code */
	return (int)ddi_power_GetBatteryChargerEnabled();
}


/*  */
/* brief Report the battery configuration. */
/*  */
/* fntype Function */
/*  */
/*  This function reports the hardware battery configuration. */
/*  */
/* retval  A value that indicates the battery configuration. */
/*  */

ddi_bc_BatteryMode_t ddi_bc_hwGetBatteryMode(void)
{
	/* TODO: replace ddi_bc_hwGetBatteryMode() with the function below. */
	return (ddi_bc_BatteryMode_t) ddi_power_GetBatteryMode();
}



/*  */
/* brief Report the voltage across the battery. */
/*  */
/* fntype Function */
/*  */
/*  This function reports the voltage across the battery. */
/*  */
/* retval The voltage across the battery, in mV. */
/*  */

uint16_t ddi_bc_hwGetBatteryVoltage(void)
{
	/* TODO: replace ddi_bc_hwGetBattery with function below */
	return ddi_power_GetBattery();
}


/*  */
/* brief Report on the presence of the power supply. */
/*  */
/* fntype Function */
/*  */
/*  This function repots on whether or not the 5V power supply is present. */
/*  */
/* retval  Zero if the power supply is not present. Non-zero otherwise. */
/*  */

int ddi_bc_hwPowerSupplyIsPresent(void)
{
	/* TODO: replace ddi_bc_hwPowerSupplyIsPresent with the functino below. */
	return (int)ddi_power_Get5vPresentFlag();
}


/*  */
/* brief Report the maximum charging current. */
/*  */
/* fntype Function */
/*  */
/*  This function reports the maximum charging current that will be offered to */
/*  the battery, as currently set in the hardware. */
/*  */
/* retval  The maximum current setting in the hardware. */
/*  */

uint16_t ddi_bc_hwGetMaxCurrent(void)
{
	/* TODO: replace ddi_bc_hwGetMaxCurrent() with the below function */
	return (uint16_t) ddi_power_GetMaxBatteryChargeCurrent();
}


/*  */
/* brief Set the maximum charging current. */
/*  */
/* fntype Function */
/*  */
/*  This function sets the maximum charging current that will be offered to the */
/*  battery. */
/*  */
/*  Note that the hardware has a minimum resolution of 10mA and a maximum */
/*  expressible value of 780mA (see the data sheet for details). If the given */
/*  current cannot be expressed exactly, then the largest expressible smaller */
/*  value will be used. The return reports the actual value that was effected. */
/*  */
/* param[in]  u16Limit  The maximum charging current, in mA. */
/*  */
/* retval  The actual value that was effected. */
/*  */

uint16_t ddi_bc_hwSetMaxCurrent(uint16_t u16Limit)
{
	/* TODO: replace ddi_bc_hwSetMaxChargeCurrent */
	return ddi_power_SetMaxBatteryChargeCurrent(u16Limit);
}


/*  */
/* brief Set the charging current threshold. */
/*  */
/* fntype Function */
/*  */
/*  This function sets the charging current threshold. When the actual current */
/*  flow to the battery is less than this threshold, the HW_POWER_STS.CHRGSTS */
/*  flag is clear. */
/*  */
/*  Note that the hardware has a minimum resolution of 10mA and a maximum */
/*  expressible value of 180mA (see the data sheet for details). If the given */
/*  current cannot be expressed exactly, then the largest expressible smaller */
/*  value will be used. The return reports the actual value that was effected. */
/*  */
/* param[in]  u16Threshold  The charging current threshold, in mA. */
/*  */
/* retval  The actual value that was effected. */
/*  */

uint16_t ddi_bc_hwSetCurrentThreshold(uint16_t u16Threshold)
{
	/* TODO: replace calls to ddi_bc_hwSetCurrentThreshold with the one below */
	return ddi_power_SetBatteryChargeCurrentThreshold(u16Threshold);

}


/*  */
/* brief Report the charging current threshold. */
/*  */
/* fntype Function */
/*  */
/*  This function reports the charging current threshold. When the actual */
/*  current flow to the battery is less than this threshold, the */
/*  HW_POWER_STS.CHRGSTS flag is clear. */
/*  */
/*  Note that the hardware has a minimum resolution of 10mA and a maximum */
/*  expressible value of 180mA (see the data sheet for details). */
/*  */
/* retval  The charging current threshold, in mA. */
/*  */

uint16_t ddi_bc_hwGetCurrentThreshold(void)
{
	/* TODO: replace calls to ddi_bc_hwGetCurrentThreshold with function below */
	return ddi_power_GetBatteryChargeCurrentThreshold();
}


/*  */
/* brief Report if the charger hardware power is on. */
/*  */
/* fntype Function */
/*  */
/*  This function reports if the charger hardware power is on. */
/*  */
/* retval  Zero if the charger hardware is not powered. Non-zero otherwise. */
/*  */

int ddi_bc_hwChargerPowerIsOn(void)
{

	/* -------------------------------------------------------------------------- */
	/* Note that the bit we're looking at is named PWD_BATTCHRG. The "PWD" */
	/* stands for "power down". Thus, when the bit is set, the battery charger */
	/* hardware is POWERED DOWN. */
	/* -------------------------------------------------------------------------- */

	/* -------------------------------------------------------------------------- */
	/* Read the register and return the result. */
	/* -------------------------------------------------------------------------- */

	/* TODO: replace ddi_bc_hwChargerPowerIsOn with function below */
	return ddi_power_GetChargerPowered();
}


/*  */
/* brief Turn the charging hardware on or off. */
/*  */
/* fntype Function */
/*  */
/*  This function turns the charging hardware on or off. */
/*  */
/* param[in]  on  Indicates whether the charging hardware should be on or off. */
/*  */

void ddi_bc_hwSetChargerPower(int on)
{

	/* -------------------------------------------------------------------------- */
	/* Note that the bit we're looking at is named PWD_BATTCHRG. The "PWD" */
	/* stands for "power down". Thus, when the bit is set, the battery charger */
	/* hardware is POWERED DOWN. */
	/* -------------------------------------------------------------------------- */

	/* -------------------------------------------------------------------------- */
	/* Hit the power switch. */
	/* -------------------------------------------------------------------------- */

	/* TODO: replace ddi_bc_hwSetChargerPower with functino below */
	ddi_power_SetChargerPowered(on);
}


/*  */
/* brief Reports if the charging current has fallen below the threshold. */
/*  */
/* fntype Function */
/*  */
/*  This function reports if the charging current that the battery is accepting */
/*  has fallen below the threshold. */
/*  */
/*  Note that this bit is regarded by the hardware guys as very slightly */
/*  unreliable. They recommend that you don't believe a value of zero until */
/*  you've sampled it twice. */
/*  */
/* retval  Zero if the battery is accepting less current than indicated by the */
/*           charging threshold. Non-zero otherwise. */
/*  */

int ddi_bc_hwGetChargeStatus(void)
{
	return ddi_power_GetChargeStatus();
}


/*  */
/* brief Report on the die temperature. */
/*  */
/* fntype Function */
/*  */
/*  This function reports on the die temperature. */
/*  */
/* param[out]  pLow   The low  end of the temperature range. */
/* param[out]  pHigh  The high end of the temperature range. */
/*  */

void ddi_bc_hwGetDieTemp(int16_t *pLow, int16_t *pHigh)
{
	/* TODO: replace ddi_bc_hwGetDieTemp with function below */
	ddi_power_GetDieTemp(pLow, pHigh);
}


/*  */
/* brief Report the battery temperature reading. */
/*  */
/* fntype Function */
/*  */
/*  This function examines the configured LRADC channel and reports the battery */
/*  temperature reading. */
/*  */
/* param[out]  pReading  A pointer to a variable that will receive the */
/*                         temperature reading. */
/*  */
/* retval  DDI_BC_STATUS_SUCCESS          If the operation succeeded. */
/* retval  DDI_BC_STATUS_NOT_INITIALIZED  If the Battery Charger is not yet */
/*                                          initialized. */
/*  */

ddi_bc_Status_t ddi_bc_hwGetBatteryTemp(uint16_t *pReading)
{
	ddi_power_GetBatteryTemp(pReading);
  return DDI_BC_STATUS_SUCCESS;
}


/*  */
/* brief Convert a current in mA to a hardware setting. */
/*  */
/* fntype Function */
/*  */
/*  This function converts a current measurement in mA to a hardware setting */
/*  used by HW_POWER_BATTCHRG.STOP_ILIMIT or HW_POWER_BATTCHRG.BATTCHRG_I. */
/*  */
/*  Note that the hardware has a minimum resolution of 10mA and a maximum */
/*  expressible value of 780mA (see the data sheet for details). If the given */
/*  current cannot be expressed exactly, then the largest expressible smaller */
/*  value will be used. */
/*  */
/* param[in]  u16Current  The current of interest. */
/*  */
/* retval  The corresponding setting. */
/*  */

uint8_t ddi_bc_hwCurrentToSetting(uint16_t u16Current)
{
	return ddi_power_convert_current_to_setting(u16Current);
}


/*  */
/* brief Convert a hardware current setting to a value in mA. */
/*  */
/* fntype Function */
/*  */
/*  This function converts a setting used by HW_POWER_BATTCHRG.STOP_ILIMIT or */
/*  HW_POWER_BATTCHRG.BATTCHRG_I into an actual current measurement in mA. */
/*  */
/*  Note that the hardware current fields are 6 bits wide. The higher bits in */
/*  the 8-bit input parameter are ignored. */
/*  */
/* param[in]  u8Setting  A hardware current setting. */
/*  */
/* retval  The corresponding current in mA. */
/*  */

uint16_t ddi_bc_hwSettingToCurrent(uint8_t u8Setting)
{
	return ddi_power_convert_setting_to_current(u8Setting);
}


/*  */
/* brief Compute the actual current expressible in the hardware. */
/*  */
/* fntype Function */
/*  */
/*  Given a desired current, this function computes the actual current */
/*  expressible in the hardware. */
/*  */
/*  Note that the hardware has a minimum resolution of 10mA and a maximum */
/*  expressible value of 780mA (see the data sheet for details). If the given */
/*  current cannot be expressed exactly, then the largest expressible smaller */
/*  value will be used. */
/*  */
/* param[in]  u16Current  The current of interest. */
/*  */
/* retval  The corresponding current in mA. */
/*  */

uint16_t ddi_bc_hwExpressibleCurrent(uint16_t u16Current)
{
	/* TODO: replace the bc function with this one */
	return ddi_power_ExpressibleCurrent(u16Current);
}


/*  */
/* brief Checks to see if the DCDC has been manually enabled */
/*  */
/* fntype Function */
/*  */
/* retval  true if DCDC is ON, false if DCDC is OFF. */
/*  */

bool ddi_bc_hwIsDcdcOn(void)
{
	return ddi_power_IsDcdcOn();
}


/* End of file */

/*  @} */
