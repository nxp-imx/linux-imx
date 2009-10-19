/*
 * Copyright 2008-2009 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

//! \brief Battery modes
typedef enum {
	// 37xx battery modes
        //! \brief LiIon battery powers the player
	DDI_POWER_BATT_MODE_LIION           = 0,
        //! \brief Alkaline/NiMH battery powers the player
	DDI_POWER_BATT_MODE_ALKALINE_NIMH   = 1,
} ddi_power_BatteryMode_t;


//! \brief Available sources for bias currents
typedef enum {
        //! \brief Use external resistor to generate bias current
	DDI_POWER_EXTERNAL_BIAS_CURRENT = 0x0,
        //! \brief Use internal resistor to generate bias current
	DDI_POWER_INTERNAL_BIAS_CURRENT = 0x1
} ddi_power_BiasCurrentSource_t;

//! \brief Possible 5V detection methods
typedef enum {
	//! \brief Use VBUSVALID comparator for detection
	DDI_POWER_5V_VBUSVALID,
	//! \brief Use VDD5V_GT_VDDIO comparison for detection
	DDI_POWER_5V_VDD5V_GT_VDDIO
} ddi_power_5vDetection_t;


uint16_t ddi_power_convert_current_to_setting(uint16_t u16Current);
uint16_t ddi_power_convert_setting_to_current(uint16_t u16Setting);
void ddi_power_enable_5v_to_battery_handoff(void);
void ddi_power_execute_5v_to_battery_handoff(void);
void ddi_power_enable_battery_to_5v_handoff(void);
void ddi_power_execute_battery_to_5v_handoff(void);
int ddi_power_init_battery(void);
ddi_power_BatteryMode_t ddi_power_GetBatteryMode(void);
bool ddi_power_GetBatteryChargerEnabled(void);
bool ddi_power_GetChargerPowered(void);
void ddi_power_SetChargerPowered(bool bPowerOn);
int ddi_power_GetChargeStatus(void);
uint16_t ddi_power_GetBattery(void);
uint16_t ddi_power_GetBatteryBrownout(void);
int ddi_power_SetBatteryBrownout(uint16_t u16BattBrownout_mV);
int ddi_power_SetBiasCurrentSource(ddi_power_BiasCurrentSource_t eSource);
ddi_power_BiasCurrentSource_t ddi_power_GetBiasCurrentSource(void);
uint16_t ddi_power_SetMaxBatteryChargeCurrent(uint16_t u16MaxCur);
uint16_t ddi_power_GetMaxBatteryChargeCurrent(void);
uint16_t ddi_power_SetBatteryChargeCurrentThreshold(uint16_t u16Thresh);
uint16_t ddi_power_GetBatteryChargeCurrentThreshold(void);
uint16_t ddi_power_ExpressibleCurrent(uint16_t u16Current);
bool ddi_power_Get5vPresentFlag(void);
void ddi_power_GetDieTemp(int16_t * pLow, int16_t * pHigh);
bool ddi_power_IsDcdcOn(void);
void ddi_power_SetPowerClkGate(bool bGate);
bool ddi_power_GetPowerClkGate(void);
