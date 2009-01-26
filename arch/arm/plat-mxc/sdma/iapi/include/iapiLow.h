/******************************************************************************
 *
 * Copyright 2007 Freescale Semiconductor, Inc. All Rights Reserved.
 *
 *
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 *
 ******************************************************************************
 *
 * File: iapiLow.h
 *
 * $Id iapiLow.h $
 *
 * Description:
 *  prototypes for low level function of I.API
 *
 *
 *
 *
 * $Log iapiLow.h
 *
 * ***************************************************************************/

#ifndef _iapiLow_h
#define _iapiLow_h

/* ****************************************************************************
 * Boolean identifiers
 *****************************************************************************/
#define NO_OS     0
#define LINUX     1
#define SYMBIAN   2
#define WINCE     3

/* ****************************************************************************
 * Include File Section
 *****************************************************************************/
#include "sdmaStruct.h"
#include "iapiDefaults.h"
#include "iapiOS.h"
#ifdef MCU
#include "iapiLowMcu.h"
#endif /*MCU*/
#if OS == NO_OS
#include <stdlib.h>
#elif OS == LINUX
#include <linux/types.h>
#endif


/* ****************************************************************************
 * Macro-command Section
 *****************************************************************************/

#define GOTO_SLEEP(x) (iapi_GotoSleep)(x)
#define INIT_SLEEP(x) (iapi_InitSleep)(x)

/* ****************************************************************************
 * Public Function Prototype Section
 *****************************************************************************/
void iapi_lowStartChannel ( unsigned char channel );
void iapi_lowStopChannel ( unsigned char channel );
void iapi_AttachCallbackISR (channelDescriptor * cd_p,
                      void (* func_p)(channelDescriptor * cd_p, void * arg));
void iapi_DetachCallbackISR (channelDescriptor * cd_p);
void iapi_ChangeCallbackISR (channelDescriptor * cd_p,
                      void (* func_p)(channelDescriptor * cd_p, void * arg));
void iapi_lowSynchChannel ( unsigned char channel );
void iapi_SetBufferDescriptor(bufferDescriptor *bd_p, unsigned char command,
                       unsigned char status, unsigned short count,
                       void * buffAddr, void * extBufferAddr);

#endif /* _iapiLow_h */
