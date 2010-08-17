/******************************************************************************
 *
 * Copyright 2007-2010 Freescale Semiconductor, Inc. All Rights Reserved.
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
 * File: iapiLowMcu.h
 *
 * $Id iapiLowMcu.h $
 *
 * Description:
 *  prototypes for low level function of I.API of MCU side only
 *
 *
 *
 *
 * $Log iapiLowMcu.h $
 *
 * ***************************************************************************/

#ifndef _iapiLowMcu_h
#define _iapiLowMcu_h

/******************************************************************************
 * Include File Section
 *****************************************************************************/
#include "sdmaStruct.h"
#include "iapiDefaults.h"

/* ****************************************************************************
 * Public Function Prototype Section
 * ***************************************************************************/

void iapi_InitChannelTables(void);
int iapi_ChannelConfig(unsigned char channel, unsigned eventOverride,
		       unsigned mcuOverride, unsigned dspOverride);
int iapi_Channel0Command(channelDescriptor *cd_p, void *buf,
			 unsigned short nbyte, unsigned char command);
void iapi_lowGetScript(channelDescriptor *cd_p, void *buf, unsigned short size,
		       unsigned long address);
void iapi_lowGetContext(channelDescriptor *cd_p, void *buf,
			unsigned char channel);
void iapi_lowSetScript(channelDescriptor *cd_p, void *buf,
		       unsigned short nbyte, unsigned long destAddr);
void iapi_lowSetContext(channelDescriptor *cd_p, void *buf,
			unsigned char channel);
int iapi_lowAssignScript(channelDescriptor *cd_p, script_data *data_p);

int iapi_lowSetChannelEventMapping(unsigned char event,
				   unsigned long channel_map);

#endif				/* _iapiLowMcu_h */
