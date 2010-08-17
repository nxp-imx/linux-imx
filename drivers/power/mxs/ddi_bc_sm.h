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
/* file ddi_bc_sm.h */
/* brief Header file for the Battery Charger state machine. */
/* date 06/2005 */
/*  */
/*  This file contains declarations for the Battery Charger state machine. */


#ifndef _DDI_BC_SM_H
#define _DDI_BC_SM_H


/* Externs */


/*  The current state. */

extern ddi_bc_State_t g_ddi_bc_State;

/*  The state function table. */

extern ddi_bc_Status_t(*const (stateFunctionTable[])) (void);


/* End of file */

#endif				/* _DDI_BC_H */
/*  @} */
