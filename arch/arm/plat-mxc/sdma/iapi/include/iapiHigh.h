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
 * File: iapiHigh.h
 *
 * $Id iapiHigh.h $
 *
 * Description:
 *  prototypes for high level function of I.API
 *
 *
 *  http://venerque.sps.mot.com/pjt/sfs/www/iapi/softsim_api.pdf
 *
 * $Log iapiHigh.h
 *
 * ***************************************************************************/

#ifndef _iapiHigh_h
#define _iapiHigh_h

/* ****************************************************************************
 * Include File Section
 *****************************************************************************/
#include "sdmaStruct.h"

/* ****************************************************************************
 * Macro-command Section
 *****************************************************************************/
enum {
  IAPI_CHANGE_CHANDESC,
  IAPI_CHANGE_BDNUM,
  IAPI_CHANGE_BUFFSIZE,
  IAPI_CHANGE_CHANBLOCK,
  IAPI_CHANGE_INSTANCE,
  IAPI_CHANGE_OWNERSHIP,
  IAPI_CHANGE_SYNCH,
  IAPI_CHANGE_TRUST,
  IAPI_CHANGE_CALLBACKFUNC,
  IAPI_CHANGE_CHANCCB,
  IAPI_CHANGE_PRIORITY,
  IAPI_CHANGE_BDWRAP,
  IAPI_CHANGE_WATERMARK,
  IAPI_CHANGE_SET_BDCONT,
  IAPI_CHANGE_UNSET_BDCONT,
  IAPI_CHANGE_SET_BDEXTD,
  IAPI_CHANGE_UNSET_BDEXTD,
  IAPI_CHANGE_EVTMASK1,
  IAPI_CHANGE_EVTMASK2,
  IAPI_CHANGE_PERIPHADDR,
  IAPI_CHANGE_SET_BDINTR,
  IAPI_CHANGE_UNSET_BDINTR,
  IAPI_CHANGE_SET_TRANSFER_CD,
  IAPI_CHANGE_FORCE_CLOSE,
  IAPI_CHANGE_SET_TRANSFER,
  IAPI_CHANGE_USER_ARG,
  IAPI_CHANGE_SET_BUFFERADDR,
  IAPI_CHANGE_SET_EXTDBUFFERADDR,
  IAPI_CHANGE_SET_COMMAND,
  IAPI_CHANGE_SET_COUNT,
  IAPI_CHANGE_SET_STATUS,
  IAPI_CHANGE_GET_BUFFERADDR,
  IAPI_CHANGE_GET_EXTDBUFFERADDR,
  IAPI_CHANGE_GET_COMMAND,
  IAPI_CHANGE_GET_COUNT,
  IAPI_CHANGE_GET_STATUS,
  IAPI_CHANGE_SET_ENDIANNESS
};


/*
 * Public Function Prototype Section
 */
int iapi_Open ( channelDescriptor * cd_p, unsigned char channelNumber );
int iapi_Close( channelDescriptor * cd_p );
int iapi_Read ( channelDescriptor * cd_p, void * buf, unsigned short nbyte );
int iapi_Write( channelDescriptor * cd_p, void * buf, unsigned short nbyte );
int iapi_MemCopy(channelDescriptor * cd_p, void* dest, void* src,
                 unsigned long size);
int iapi_IoCtl( channelDescriptor * cd_p, unsigned long ctlRequest,
				unsigned long param);


int iapi_Read_ipcv2( channelDescriptor * cd_p, void * data_control_struct_ipcv2);

int iapi_Write_ipcv2( channelDescriptor * cd_p, void * data_control_struct_ipcv2);

#ifdef MCU
int iapi_Init(channelDescriptor * cd_p, configs_data * config_p,
              unsigned short* ram_image, unsigned short code_size,
              unsigned long start_addr);
#endif /* MCU */
#ifdef DSP
int iapi_Init(channelDescriptor * cd_p);
#endif /* DSP */

int iapi_StartChannel(unsigned char channel);
int iapi_StopChannel(unsigned char channel);
int iapi_SynchChannel(unsigned char channel);

int iapi_GetChannelNumber(channelDescriptor * cd_p);
unsigned long iapi_GetError(channelDescriptor * cd_p);
int iapi_GetCount(channelDescriptor * cd_p);
int iapi_GetCountAll(channelDescriptor * cd_p);

#ifndef IRQ_KEYWORD
#define IRQ_KEYWORD
#endif /* IRQ_KEYWORD */

IRQ_KEYWORD void IRQ_Handler(void);

#ifdef MCU
int iapi_GetScript(channelDescriptor * cd_p, void * buf, unsigned short size,
                    unsigned long address);
int iapi_GetContext(channelDescriptor * cd_p, void * buf,
                     unsigned char channel);
int iapi_SetScript(channelDescriptor * cd_p, void * buf, unsigned short nbyte,
                     unsigned long destAddr);
int iapi_SetContext(channelDescriptor * cd_p, void * buf,
                     unsigned char channel);
int iapi_AssignScript(channelDescriptor * cd_p, script_data * data_p);

int iapi_SetChannelEventMapping(unsigned char event, unsigned long channel_map);
#endif /* MCU */

#endif /* _iapiHigh_h */
