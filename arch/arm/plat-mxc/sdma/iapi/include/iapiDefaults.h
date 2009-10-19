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
 * File: iapiDefaults.h
 *
 * $Id iapiDefaults.h $
 *
 * Description:
 *  This library is written in C to guarantee functionality and integrity in
 * the usage of SDMA virtual DMA channels. This API (Application Programming
 * Interface)  allow SDMA channels' access in an OPEN, READ, WRITE, CLOSE
 * fashion.
 *
 *
 *
 *
 * $Log iapiDefaults.h $
 *
 *****************************************************************************/


#ifndef _iapi_defaults_h
#define _iapi_defaults_h

/******************************************************************************
 * Include File Section
 *****************************************************************************/
#include "epm.h"
#include "sdmaStruct.h"

/* ****************************************************************************
 * Macro-command Section
 * ***************************************************************************/

/**
 * Error codes
 * lower 5 bits free to include channel number when available
 * and bit number 6 must be set when channel number is available
 *
 * Note :
 * 1) Abbreviations / naming convention :
 *    - BD  : Buffer Descriptor
 *    - CC  : Channel Context
 *    - CCB : Channel Control Block
 *    - CD  : Channel Descriptor
 *    - B   : Buffer
 *    - CH  : Channel
 *
 */
#define IAPI_SUCCESS 0
#define IAPI_FAILURE -1
#define IAPI_ERR_CH_AVAILABLE          0x00020
#define IAPI_ERR_NO_ERROR              0x00000
#define IAPI_ERR_NO_CCB_DEFINED        0x01000
#define IAPI_ERR_BD_UNINITIALIZED      0x02000
#define IAPI_ERR_BD_ALLOCATED          0x03000
#define IAPI_ERR_BD_ALLOCATION         0x04000
#define IAPI_ERR_CCB_ALLOC_FAILED      0x05000
#define IAPI_ERR_CCB_UNINITIALIZED     0x06000
#define IAPI_ERR_CC_ALREADY_DEFINED    0x07000
#define IAPI_ERR_CC_ALLOC_FAILED       0x08000
#define IAPI_ERR_CD_ALREADY_DEFINED    0x09000
#define IAPI_ERR_CD_ALLOC_FAILED       0x0A000
#define IAPI_ERR_CD_CHANGE_CH_NUMBER   0x0B000
#define IAPI_ERR_CD_CHANGE_CCB_PTR     0x0C000
#define IAPI_ERR_CD_CHANGE_UNKNOWN     0x0D000
#define IAPI_ERR_CD_CHANGE             0x0E000
#define IAPI_ERR_CD_UNINITIALIZED      0x0F000
#define IAPI_ERR_CLOSE                 0x10000
#define IAPI_ERR_B_ALLOC_FAILED        0x11000
#define IAPI_ERR_CONFIG_OVERRIDE       0x12000
#define IAPI_ERR_CH_IN_USE             0x13000
#define IAPI_ERR_CALLBACKSYNCH_UNKNOWN 0x14000
#define IAPI_ERR_INVALID_PARAMETER     0x15000
#define IAPI_ERR_TRUST                 0x16000
#define IAPI_ERR_CHANNEL_UNINITIALIZED 0x17000
#define IAPI_ERR_RROR_BIT_READ         0x18000
#define IAPI_ERR_RROR_BIT_WRITE        0x19000
#define IAPI_ERR_NOT_ALLOWED           0x1A000
#define IAPI_ERR_NO_OS_FN              0x1B000


/*
 * Global Variable Section
 */

/*
 * Table to hold pointers to the callback functions registered by the users of
 *I.API
 */
extern void (* callbackIsrTable[CH_NUM])(channelDescriptor * cd_p, void * arg);

/*
 * Table to hold user registered data pointers, to be privided in the callback
 *function
 */
extern void * userArgTable[CH_NUM];

/* channelDescriptor data structure filled with default data*/
extern channelDescriptor iapi_ChannelDefaults;

/* Global variable to hold the last error encountered in I.API operations*/
extern unsigned int iapi_errno;

/* Used in synchronization, to mark started channels*/
extern volatile unsigned long iapi_SDMAIntr;

/* Hold a pointer to the start of the CCB array, to be used in the IRQ routine
 *to find the channel descriptor for the channed sending the interrupt to the
 *core.
 */
extern channelControlBlock * iapi_CCBHead;

/* configs_data structure filled with default data*/
extern configs_data iapi_ConfigDefaults;

#endif /* iapiDefaults_h */
