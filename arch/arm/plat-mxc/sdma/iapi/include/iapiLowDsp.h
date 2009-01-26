/*
 * Copyright 2007 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

/* ****************************************************************************
 *
 * File: iapiLowDsp.h
 *
 * $Id iapiLowDsp.h $
 *
 * Description:
 *  prototypes for low level function of I.API for DSP side only
 *
 *
 *
 *
 * $Log iapiLowDsp.h
 *
 * ***************************************************************************/

#ifndef _iapiLowDsp_h
#define _iapiLowDsp_h

/* ****************************************************************************
 * Include File Section
 *****************************************************************************/


/* ****************************************************************************
 * Public Function Prototype Section
 *****************************************************************************/
/* WARNING !!!!!
 * This file is empty and it is normal, because there is no low level functions
 * dedicated to the DSP but the file (iapi_LowDsp.h) must still exist because
 * some project directly links the file. Previously, there were function
 * iapi_EnableInterrupts,iapi_DisableInterrupts,iapi_WaitCore,iapi_StartChannel
 * iapi_StopChannel but they are common to both MCU and DSP, so they have been
 * moved to iapi_Low.h file.
 */

#endif /* _iapiLowDsp_h */
