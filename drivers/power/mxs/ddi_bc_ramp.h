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
/* ! @{ */
/*  */
/* Copyright (c) 2004-2005 SigmaTel, Inc. */
/*  */
/* file ddi_bc_ramp.h */
/* brief Internal header file for Battery Charger current ramp controller. */
/* date 06/2005 */
/* ! */
/* ! This file contains internal declarations for Battery current ramp */
/* ! controller. */


#ifndef _DDI_BC_RAMP_H
#define _DDI_BC_RAMP_H


/* Prototypes */


extern void ddi_bc_RampReset(void);
extern uint16_t ddi_bc_RampSetTarget(uint16_t);
extern uint16_t ddi_bc_RampGetTarget(void);
extern uint16_t ddi_bc_RampSetLimit(uint16_t);
extern uint16_t ddi_bc_RampGetLimit(void);
extern void ddi_bc_RampUpdateAlarms(void);
extern int ddi_bc_RampGetDieTempAlarm(void);
extern int ddi_bc_RampGetBatteryTempAlarm(void);
extern int ddi_bc_RampGetAmbientTempAlarm(void);
extern ddi_bc_Status_t ddi_bc_RampStep(uint32_t);


/* End of file */

#endif				/* _DDI_BC_H */
/* ! @} */
