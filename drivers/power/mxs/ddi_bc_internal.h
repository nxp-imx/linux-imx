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
/* file ddi_bc_internal.h */
/* brief Internal header file for the Battery Charger device driver. */
/* date 06/2005 */
/*  */
/*  This file contains internal declarations for the Battery Charger device */
/*  driver. */


#ifndef _DDI_BC_INTERNAL_H
#define _DDI_BC_INTERNAL_H


/* Includes */


#include <mach/ddi_bc.h>
#include "ddi_bc_hw.h"
#include "ddi_bc_ramp.h"
#include "ddi_bc_sm.h"
#include "ddi_power_battery.h"


/* Externs */

#include <linux/kernel.h>

extern bool g_ddi_bc_Configured;
extern ddi_bc_Cfg_t g_ddi_bc_Configuration;


/* End of file */

#endif				/* _DDI_BC_H */
/*  @} */
