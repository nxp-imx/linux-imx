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
 * File: iapiMiddle.h
 *
 * $Id iapiMiddle.h $
 *
 * Description:
 *  prototypes for middle level function of I.API
 *
 *
 *
 *
 * $Log iapiMiddle.h
 *
 * ***************************************************************************/

#ifndef _iapiMiddle_h
#define _iapiMiddle_h

/* ****************************************************************************
 * Include File Section
 ******************************************************************************/
#include "sdmaStruct.h"
#ifdef MCU
#include "iapiMiddleMcu.h"
#endif /* MCU */

/* ****************************************************************************
 * Public Function Prototype Section
 ******************************************************************************/
bufferDescriptor * iapi_AllocBD (channelControlBlock * ccb_p);
int  iapi_AllocContext(contextData ** ctxd_p, unsigned char channel);
int  iapi_AllocChannelDesc(channelDescriptor ** cd_p, unsigned char channel);
int  iapi_ChangeChannelDesc (channelDescriptor * cd_p,
							unsigned char whatToChange,	unsigned long newval);
void iapi_InitializeCallbackISR (void (* func_p)(channelDescriptor * cd_p,
                                 void * arg));
int iapi_InitializeMemory (channelControlBlock * ccb_p);

#endif /* iapiMiddle_h */
