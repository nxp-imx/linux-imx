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

/* brief Battery modes */
typedef enum {
	/* 37xx battery modes */
	/* brief LiIon battery powers the player */
	DDI_POWER_BATT_MODE_LIION           = 0,
	/* brief Alkaline/NiMH battery powers the player */
	DDI_POWER_BATT_MODE_ALKALINE_NIMH   = 1,
} ddi_power_BatteryMode_t;


/* brief Possible 5V detection methods */
typedef enum {
	/* brief Use VBUSVALID comparator for detection */
	DDI_POWER_5V_VBUSVALID,
	/* brief Use VDD5V_GT_VDDIO comparison for detection */
	DDI_POWER_5V_VDD5V_GT_VDDIO
} ddi_power_5vDetection_t;


enum ddi_power_5v_status {
	new_5v_connection,
	existing_5v_connection,
	new_5v_disconnection,
	existing_5v_disconnection,
} ;


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
uint16_t ddi_power_SetMaxBatteryChargeCurrent(uint16_t u16MaxCur);
uint16_t ddi_power_GetMaxBatteryChargeCurrent(void);
uint16_t ddi_power_SetBatteryChargeCurrentThreshold(uint16_t u16Thresh);
uint16_t ddi_power_GetBatteryChargeCurrentThreshold(void);
uint16_t ddi_power_ExpressibleCurrent(uint16_t u16Current);
bool ddi_power_Get5vPresentFlag(void);
void ddi_power_GetDieTemp(int16_t *pLow, int16_t *pHigh);
bool ddi_power_IsDcdcOn(void);
void ddi_power_SetPowerClkGate(bool bGate);
bool ddi_power_GetPowerClkGate(void);
enum ddi_power_5v_status ddi_power_GetPmu5vStatus(void);
void ddi_power_EnableBatteryBoFiq(bool bEnable);
void ddi_power_disable_5v_connection_irq(void);
void ddi_power_enable_5v_disconnect_detection(void);
void ddi_power_enable_5v_connect_detection(void);
void ddi_power_Enable5vDisconnectShutdown(bool bEnable);
void ddi_power_enable_5v_to_battery_xfer(bool bEnable);
void ddi_power_init_4p2_protection(void);
bool ddi_power_check_4p2_bits(void);
void ddi_power_Start4p2Dcdc(bool battery_ready);
void ddi_power_Init4p2Params(void);
bool ddi_power_IsBattRdyForXfer(void);
void ddi_power_EnableVbusDroopIrq(void);
void ddi_power_Enable4p2(uint16_t target_current_limit_ma);
uint16_t ddi_power_BringUp4p2Regulator(
		uint16_t target_current_limit_ma,
		bool b4p2_dcdc_enabled);
void ddi_power_Set4p2BoLevel(uint16_t bo_voltage_mv);
void ddi_power_EnableBatteryBoInterrupt(bool bEnable);
void ddi_power_handle_cmptrip(void);
uint16_t ddi_power_set_4p2_ilimit(uint16_t ilimit);
void ddi_power_shutdown(void);
void ddi_power_handle_dcdc4p2_bo(void);
void ddi_power_enable_vddio_interrupt(bool enable);
void ddi_power_handle_vddio_brnout(void);
void ddi_power_EnableDcdc4p2BoInterrupt(bool bEnable);
void ddi_power_handle_vdd5v_droop(void);
void ddi_power_InitOutputBrownouts(void);
void ddi_power_disable_power_interrupts(void);
bool ddi_power_Get5vDroopFlag(void);
