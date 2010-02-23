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

#ifndef _DDI_BC_HW_H
#define _DDI_BC_HW_H


/* Definitions */


/*  The enumeration of battery modes. */

typedef enum _ddi_bc_BatteryMode {
	DDI_BC_BATTERY_MODE_LI_ION_2_CELLS = 0,
	DDI_BC_BATTERY_MODE_LI_ION_1_CELL = 1,
	DDI_BC_BATTERY_MODE_2_CELLS = 2,
	DDI_BC_BATTERY_MODE_1_CELL = 3
} ddi_bc_BatteryMode_t;

/*  The enumeration of bias current sources. */

typedef enum _ddi_bc_BiasCurrentSource {
	DDI_BC_EXTERNAL_BIAS_CURRENT = 0,
	DDI_BC_INTERNAL_BIAS_CURRENT = 1,
} ddi_bc_BiasCurrentSource_t;


/* Prototypes */


extern int ddi_bc_hwBatteryChargerIsEnabled(void);
extern ddi_bc_BatteryMode_t ddi_bc_hwGetBatteryMode(void);
extern ddi_bc_BiasCurrentSource_t ddi_bc_hwGetBiasCurrentSource(void);
extern ddi_bc_Status_t
ddi_bc_hwSetBiasCurrentSource(ddi_bc_BiasCurrentSource_t);
extern ddi_bc_Status_t ddi_bc_hwSetChargingVoltage(uint16_t);
extern uint16_t ddi_bc_hwGetBatteryVoltage(void);
extern int ddi_bc_hwPowerSupplyIsPresent(void);
extern uint16_t ddi_bc_hwSetMaxCurrent(uint16_t);
extern uint16_t ddi_bc_hwGetMaxCurrent(void);
extern uint16_t ddi_bc_hwSetCurrentThreshold(uint16_t);
extern uint16_t ddi_bc_hwGetCurrentThreshold(void);
extern int ddi_bc_hwChargerPowerIsOn(void);
extern void ddi_bc_hwSetChargerPower(int);
extern int ddi_bc_hwGetChargeStatus(void);
extern void ddi_bc_hwGetDieTemp(int16_t *, int16_t *);
extern ddi_bc_Status_t ddi_bc_hwGetBatteryTemp(uint16_t *);
uint8_t ddi_bc_hwCurrentToSetting(uint16_t);
uint16_t ddi_bc_hwSettingToCurrent(uint8_t);
uint16_t ddi_bc_hwExpressibleCurrent(uint16_t);


/*  */
/* brief Checks to see if the DCDC has been manually enabled */
/*  */
/* fntype Function */
/*  */
/* retval  true if DCDC is ON, false if DCDC is OFF. */
/*  */

bool ddi_bc_hwIsDcdcOn(void);


/* End of file */

#endif				/* _DDI_BC_H */
/*  @} */
